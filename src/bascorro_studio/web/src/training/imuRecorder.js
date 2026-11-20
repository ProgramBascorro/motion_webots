import ROSLIB from "roslib";

// Perekam /robotis/open_cr/imu untuk tab training. Sampel = [t, gx, gy, gz,
// qx, qy, qz, qw]; t = cap waktu header (detik), jadi semua tanda waktu
// percobaan (rec.last()) ikut jam yang sama dengan sampelnya. Dibulatkan supaya
// rekaman yang dikirim ke apply_node (act_metrics) tetap kecil.
// onSample dipanggil tiap sampel (rem darurat Action Training memakainya).

const round = (v, digits) => Number(v.toFixed(digits));

export function recordImu(ros, onSample) {
  const rec = { samples: [] };
  const topic = new ROSLIB.Topic({
    ros, name: "/robotis/open_cr/imu", messageType: "sensor_msgs/Imu", throttle_rate: 0, queue_length: 0,
  });
  let stopped = false;
  topic.subscribe((msg) => {
    if (stopped) return;
    const stamp = msg.header?.stamp;
    const t = stamp?.sec ? stamp.sec + (stamp.nanosec || 0) * 1e-9 : performance.now() / 1000;
    const q = msg.orientation;
    const g = msg.angular_velocity;
    const sample = [round(t, 4), round(g.x, 5), round(g.y, 5), round(g.z, 5),
      round(q.x, 6), round(q.y, 6), round(q.z, 6), round(q.w, 6)];
    rec.samples.push(sample);
    onSample?.(sample);
  });
  rec.last = () => (rec.samples.length ? rec.samples[rec.samples.length - 1][0] : 0);
  rec.stop = () => {
    if (stopped) return;
    stopped = true;
    topic.unsubscribe();
  };
  return rec;
}
