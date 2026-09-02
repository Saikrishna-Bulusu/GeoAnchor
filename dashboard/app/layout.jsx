import './globals.css';

export const metadata = {
  title: 'GeoAnchor',
  description: 'GNSS-denied absolute visual localization — live telemetry',
};

export default function RootLayout({ children }) {
  return (
    <html lang="en">
      <body>{children}</body>
    </html>
  );
}
