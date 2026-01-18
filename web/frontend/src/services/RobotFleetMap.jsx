import React, { useMemo } from "react";
import { MapContainer, TileLayer, CircleMarker, Tooltip } from "react-leaflet";
import "leaflet/dist/leaflet.css";

export default function RobotFleetMap({ robots = [], selectedRobot, onSelectRobot }) {
  if (!robots || robots.length === 0) return null;

  // Determine center: selected robot or mean of robots
  const center = useMemo(() => {
    if (selectedRobot) return [selectedRobot.lat, selectedRobot.lng];
    const avgLat = robots.reduce((s, r) => s + r.lat, 0) / robots.length;
    const avgLng = robots.reduce((s, r) => s + r.lng, 0) / robots.length;
    return [avgLat, avgLng];
  }, [robots, selectedRobot]);

  // color by status
  const statusColor = (status) => {
    if (!status) return "#888";
    const s = status.toLowerCase();
    if (s === "active") return "#22c55e"; // green
    if (s === "idle") return "#fbbf24"; // yellow
    if (s === "error" || s === "inactive") return "#ef4444"; // red
    return "#6b7280"; // gray
  };

  return (
    <MapContainer
      center={center}
      zoom={17}
      style={{ height: "100%", width: "100%", borderRadius: "12px" }}
      scrollWheelZoom={false}
    >
      <TileLayer url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" />

      {robots.map((robot) => {
        const isSelected = selectedRobot?.id === robot.id;
        return (
          <CircleMarker
            key={robot.id}
            center={[robot.lat, robot.lng]}
            radius={isSelected ? 10 : 7}
            pathOptions={{
              color: statusColor(robot.status),
              fillColor: statusColor(robot.status),
              fillOpacity: 0.9,
              weight: isSelected ? 3 : 1,
            }}
            eventHandlers={{
              click: () => onSelectRobot(robot),
            }}
          >
            {/* permanent label next to the marker */}
            <Tooltip 
              direction="top" 
              offset={[0, -10]}
              permanent
              opacity={0.95}
              className="robot-label"
            >
              <div className="robot-label-title">{robot.name}</div>
            </Tooltip>

          </CircleMarker>
        );
      })}
    </MapContainer>
  );
}
