import './globals.css';

export const metadata = {
  title: 'GeoAnchor',
  description: 'GNSS-denied absolute visual localization — live telemetry',
};

/**
 * Without this the page has NO viewport meta tag, so a phone lays it out at a
 * virtual 980 px and scales the result down: every label becomes unreadable
 * and the whole page is a pinch-zoom target. That was the single biggest
 * mobile fault here and it is one export.
 *
 * `maximumScale` is deliberately not set. Locking zoom is an accessibility
 * failure, and a field operator squinting at a 4 px inlier count on a phone in
 * sunlight is exactly the person who needs to pinch in.
 */
export const viewport = {
  width: 'device-width',
  initialScale: 1,
  viewportFit: 'cover',
  themeColor: '#0d1117',
};

export default function RootLayout({ children }) {
  return (
    <html lang="en">
      <body>{children}</body>
    </html>
  );
}
