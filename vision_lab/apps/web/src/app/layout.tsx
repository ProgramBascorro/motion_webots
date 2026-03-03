import type { Metadata } from "next";

import "./globals.css";

export const metadata: Metadata = {
  title: "Bascorro Studio Vision Lab",
  description: "Visual CV pipeline debugger for robotics experiments",
};

export default function RootLayout({ children }: Readonly<{ children: React.ReactNode }>) {
  return (
    <html lang="en">
      <body>{children}</body>
    </html>
  );
}
