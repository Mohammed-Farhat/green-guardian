import { MapContainer, TileLayer, Marker, Polyline, Popup } from "react-leaflet";
import L from "leaflet";
import "leaflet/dist/leaflet.css";

// Fix default marker icons in bundlers (Vite, CRA, etc.)
delete L.Icon.Default.prototype._getIconUrl;
L.Icon.Default.mergeOptions({
  iconRetinaUrl: new URL(
    "leaflet/dist/images/marker-icon-2x.png",
    import.meta.url
  ).toString(),
  iconUrl: new URL(
    "leaflet/dist/images/marker-icon.png",
    import.meta.url
  ).toString(),
  shadowUrl: new URL(
    "leaflet/dist/images/marker-shadow.png",
    import.meta.url
  ).toString(),
});

export default function RobotMap({ lat, lng, history = [] }) {
  const center = [lat, lng];

  // Build path from history
  const path = history.map((p) => [p.lat, p.lng]);

  return (
    <MapContainer
      center={center}
      zoom={18}
      style={{ height: "320px", width: "100%", borderRadius: "12px" }}
      scrollWheelZoom={true}
    >
      <TileLayer
        // Free OpenStreetMap tiles (no API key needed)
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        attribution='&copy; OpenStreetMap contributors'
      />

      {/* Robot marker */}
      <Marker position={center}>
        <Popup>
          Robot position
          <br />
          {lat}, {lng}
        </Popup>
      </Marker>

      {/* Optional: draw the recent path */}
      {path.length > 1 && (
        <Polyline positions={path} color="#2563eb" weight={3} />
      )}
    </MapContainer>
  );
}
