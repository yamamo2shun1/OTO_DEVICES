import type { Metadata } from "next";
import { Geist, Geist_Mono } from "next/font/google";
import "./globals.css";

const geistSans = Geist({ variable: "--font-geist-sans", subsets: ["latin"] });
const geistMono = Geist_Mono({ variable: "--font-geist-mono", subsets: ["latin"] });

export const metadata: Metadata = {
  metadataBase: new URL(process.env.SITE_URL ?? "http://localhost:3000"),
  title: "JUMBLEQ Configurator",
  description: "Configure routing, controls, and channel-fader response for JUMBLEQ.",
  icons: { icon: "/favicon.svg", shortcut: "/favicon.svg" },
  openGraph: {
    title: "JUMBLEQ Configurator",
    description: "Shape routing, controls, and channel-fader response from any supported device.",
    images: [{ url: "/og.png", width: 1732, height: 909, alt: "JUMBLEQ Configurator" }],
  },
  twitter: {
    card: "summary_large_image",
    title: "JUMBLEQ Configurator",
    description: "Shape routing, controls, and channel-fader response from any supported device.",
    images: ["/og.png"],
  },
};

export default function RootLayout({ children }: Readonly<{ children: React.ReactNode }>) {
  return <html lang="en"><body className={`${geistSans.variable} ${geistMono.variable}`}>{children}</body></html>;
}
