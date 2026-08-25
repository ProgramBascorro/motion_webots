#!/usr/bin/env python3
"""Tiru loop kontrol robotis_controller di luar ROS, untuk memisahkan salah bus
dari salah perangkat lunak.

Kenapa alat ini ada (2026-08-25): gerakan kepala tersendat berbulan-bulan dan
tuduhannya selalu ke tempat yang salah -- servo, kabel, gain, beban CPU, YOLO.
Skrip ini menjalankan pola yang PERSIS sama dengan robotis_controller (sync
write lalu bulk read, 125 Hz, tcflush sebelum tiap tulis seperti yang dilakukan
DynamixelSDK) tanpa ROS sama sekali. Kalau skrip ini 100% dan manager tidak,
maka busnya sehat dan yang salah ada di penjadwalan -- bukan di perangkat keras.

Itulah yang terjadi: skrip ini 100% selama 20 detik, sementara manager 0%.
Yang membuka kasusnya adalah --opencr: menambahkan port kedua ke loop yang sama
langsung meruntuhkan bacaan servo, karena OpenCR full-speed 12 Mbit menempel di
hub USB yang sama dengan U2D2 dan split transaction-nya menggerus jendela
balasan servo. --cr-early membuktikan obatnya (transaksi sensor dipindah ke awal
siklus), dan itulah yang sekarang ada di robotis_controller.cpp.

WAJIB: matikan op3_manager dulu. Dua penulis di satu bus akan merusak bulk read
dan membuat present_joint_states terbaca -3,14159 rad semua.

    scripts/op3_bus_loop.py                      # baseline, satu port
    scripts/op3_bus_loop.py --opencr             # tiru dua port (gejala muncul)
    scripts/op3_bus_loop.py --opencr --cr-early  # dengan perbaikannya
    scripts/op3_bus_loop.py --opencr --cr-every 4
    scripts/op3_bus_loop.py --goal               # tulis goal_position, bukan LED
                                                 # (nilainya = posisi sekarang,
                                                 #  jadi robot tidak bergerak)

Membaca hasilnya: kolom "20 servo lengkap" adalah persentase siklus yang berhasil
mengumpulkan status dari SEMUA 20 servo dengan CRC benar. "byte@rx" berapa byte
yang sudah tersedia saat giliran baca tiba (420 kalau utuh, atau 300 kalau
BULK READ ITEMS sudah diramping ke present_position saja).
"""
import os, sys, time, termios, argparse

USB="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0"
ACM="/dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00"

def crc16(d):
    c=0
    for b in d:
        c ^= (b<<8)
        for _ in range(8):
            c = ((c<<1)^0x8005)&0xFFFF if c&0x8000 else (c<<1)&0xFFFF
    return c
def mkpkt(pid,inst,params):
    ln=len(params)+3
    p=bytes([0xFF,0xFF,0xFD,0x00,pid,ln&0xFF,(ln>>8)&0xFF,inst])+bytes(params)
    c=crc16(p); return p+bytes([c&0xFF,(c>>8)&0xFF])
def bulk(entries):
    prm=[]
    for i,a,l in entries: prm+=[i,a&0xFF,(a>>8)&0xFF,l&0xFF,(l>>8)&0xFF]
    return mkpkt(0xFE,0x92,prm)
def syncw(addr,dlen,items):
    prm=[addr&0xFF,(addr>>8)&0xFF,dlen&0xFF,(dlen>>8)&0xFF]
    for i,v in items:
        prm.append(i)
        for k in range(dlen): prm.append((v>>(8*k))&0xFF)
    return mkpkt(0xFE,0x83,prm)
def open_port(path,baud=2000000):
    fd=os.open(path,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
    a=termios.tcgetattr(fd); sp=getattr(termios,"B%d"%baud)
    a[0]=0;a[1]=0;a[2]=termios.CS8|termios.CREAD|termios.CLOCAL;a[3]=0
    a[4]=a[5]=sp;a[6]=list(a[6]);a[6][termios.VMIN]=0;a[6][termios.VTIME]=0
    termios.tcsetattr(fd,termios.TCSANOW,a); termios.tcflush(fd,termios.TCIOFLUSH)
    return fd
def rd(fd,n=8192):
    try: return os.read(fd,n)
    except BlockingIOError: return b""
def wr(fd,data):
    off=0; t0=time.perf_counter()
    while off<len(data):
        try: off+=os.write(fd,data[off:])
        except BlockingIOError:
            if time.perf_counter()-t0>0.05: return False
    return True

ap=argparse.ArgumentParser()
ap.add_argument("--secs",type=float,default=30)
ap.add_argument("--timeout",type=float,default=1.0)
ap.add_argument("--opencr",action="store_true",help="ikut baca port OpenCR di loop yang sama")
ap.add_argument("--goal",action="store_true",help="sync write goal_position (bukan LED)")
ap.add_argument("--cr-noflush",action="store_true",help="jangan tcflush port OpenCR")
ap.add_argument("--cr-every",type=int,default=1,help="sentuh OpenCR tiap N siklus")
ap.add_argument("--cr-idle",action="store_true",help="buka port OpenCR tapi jangan dipakai")
ap.add_argument("--cr-first",action="store_true",help="Tx OpenCR SEBELUM Tx bulk read servo")
ap.add_argument("--cr-early",action="store_true",help="seluruh transaksi OpenCR di AWAL siklus")
ap.add_argument("--short",action="store_true",help="bulk read 4 byte/servo, bukan 10")
ap.add_argument("--no-usb-flush",action="store_true",help="JANGAN buang isi buffer servo sebelum menulis")
a=ap.parse_args()

IDS=list(range(1,21))
NB = 4 if a.short else 10
EXPECT=20*(11+NB)
BR=bulk([(i,132,NB) for i in IDS])
BR_CR=bulk([(200,30,32)]); EXPECT_CR=43

fd=open_port(USB)
fdc=open_port(ACM) if a.opencr else None

# ---- kalau menulis goal_position: pakai posisi SEKARANG supaya tidak bergerak
if a.goal:
    termios.tcflush(fd,termios.TCIOFLUSH); wr(fd,BR)
    t0=time.perf_counter(); buf=b""
    while time.perf_counter()-t0<0.05 and len(buf)<EXPECT:
        d=rd(fd)
        if d: buf+=d
    if len(buf)<EXPECT:
        print("GAGAL membaca posisi awal, batal (tidak aman menulis goal)"); sys.exit(1)
    pos={}
    off=0
    PL=11+NB
    while off+PL<=len(buf):
        p=buf[off:off+PL]
        if p[0:4]!=b"\xff\xff\xfd\x00": off+=1; continue
        pid=p[4]; data=p[9:9+NB]
        pos[pid]=data[0]|(data[1]<<8)|(data[2]<<16)|(data[3]<<24)
        off+=PL
    if len(pos)<20:
        print(f"cuma {len(pos)} servo terbaca, batal"); sys.exit(1)
    SW=syncw(116,4,[(i,pos[i]) for i in IDS])
    print(f"  (menulis kembali posisi sekarang ke {len(pos)} servo -- tidak ada gerakan)")
else:
    SW=syncw(65,1,[(i,0) for i in IDS])

CYCLE=0.008
t_next=time.perf_counter()+CYCLE
carry=b""
ok=n=av=0; okc=nc=0; t_sec=time.perf_counter(); sec=0
wr(fd,BR)
if fdc and not a.cr_idle: wr(fdc,BR_CR)
end=time.perf_counter()+a.secs
while time.perf_counter()<end:
    now=time.perf_counter()
    if now<t_next: time.sleep(max(0,t_next-now))
    t_next+=CYCLE
    if a.cr_early and fdc is not None and not a.cr_idle and (n % a.cr_every == 0):
        t0=time.perf_counter(); b2=rd(fdc)
        while len(b2)<EXPECT_CR and (time.perf_counter()-t0)*1000<a.timeout:
            d=rd(fdc)
            if d: b2+=d
        nc+=1
        if len(b2)>=EXPECT_CR: okc+=1
        if not a.cr_noflush: termios.tcflush(fdc,termios.TCIFLUSH)
        wr(fdc,BR_CR)

    # --- BulkRead Rx: servo dulu, lalu OpenCR (urutan map di controller) ---
    t0=time.perf_counter()
    carry += rd(fd)
    av += len(carry)
    while len(carry)<EXPECT and (time.perf_counter()-t0)*1000<a.timeout:
        d=rd(fd)
        if d: carry+=d
    # ambil paket status yang lengkap & CRC benar; sisa byte dibawa ke siklus berikut
    seen=set(); off=0; PLEN=11+NB
    while off+PLEN<=len(carry):
        if carry[off:off+4]!=b"\xff\xff\xfd\x00":
            off+=1; continue
        pk=carry[off:off+PLEN]
        c=pk[PLEN-2]|(pk[PLEN-1]<<8)
        if crc16(pk[:PLEN-2])==c and pk[7]==0x55:
            seen.add(pk[4]); off+=PLEN
        else:
            off+=1
    carry = carry[off:]
    if len(carry) > 4096: carry = b""       # jaring pengaman, jangan tumbuh liar
    n+=1
    if len(seen)>=20: ok+=1
    cyc = n
    do_cr = fdc is not None and not a.cr_idle and not a.cr_early and (cyc % a.cr_every == 0)
    if do_cr:
        t0=time.perf_counter(); b2=rd(fdc)
        while len(b2)<EXPECT_CR and (time.perf_counter()-t0)*1000<a.timeout:
            d=rd(fdc)
            if d: b2+=d
        nc+=1
        if len(b2)>=EXPECT_CR: okc+=1
    # --- SyncWrite ---
    if not a.no_usb_flush: termios.tcflush(fd,termios.TCIFLUSH)
    wr(fd,SW)
    # --- BulkRead Tx ---
    if do_cr and a.cr_first and not a.cr_early:
        if not a.cr_noflush: termios.tcflush(fdc,termios.TCIFLUSH)
        wr(fdc,BR_CR)
    if not a.no_usb_flush: termios.tcflush(fd,termios.TCIFLUSH)
    wr(fd,BR)
    if do_cr and not a.cr_first and not a.cr_early:
        if not a.cr_noflush: termios.tcflush(fdc,termios.TCIFLUSH)
        wr(fdc,BR_CR)
    if time.perf_counter()-t_sec>=1.0:
        sec+=1
        extra=f"  opencr={100.0*okc/max(1,nc):5.1f}%" if fdc else ""
        print(f"  dtk {sec:2d}: 20 servo lengkap={100.0*ok/max(1,n):5.1f}% ({ok}/{n}) byte@rx={av//max(1,n)} sisa={len(carry)}{extra}")
        sys.stdout.flush()
        ok=n=av=okc=nc=0; t_sec=time.perf_counter()
os.close(fd)
if fdc: os.close(fdc)
