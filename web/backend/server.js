require("dotenv").config();

const REQUIRED_ENV = ["JWT_SECRET", "MONGODB_URI"];
const missing = REQUIRED_ENV.filter((k) => !process.env[k]);
if (missing.length) {
  console.error("Missing required environment variables:", missing.join(", "));
  process.exit(1);
}

const express = require("express");
const cors = require("cors");
const connectDB = require("./config/db");
const rosBridgeService = require("./services/rosBridgeService");
const novncService = require("./services/novncService");
const telemetryRoutes = require("./routes/telemetryRoutes");
const robotRoutes = require("./routes/robotRoutes");
const authRoutes = require("./routes/authRoutes");
const { auth } = require("./middleware/auth");

const app = express();

app.use(
  cors({
    origin: process.env.ALLOWED_ORIGINS
      ? process.env.ALLOWED_ORIGINS.split(",").map((o) => o.trim())
      : "*",
  })
);
app.use(express.json());

app.use("/api/auth", authRoutes);
app.use("/api/telemetry", telemetryRoutes);
app.use("/api/robot", auth, robotRoutes);

app.get("/api/health", (req, res) => {
  res.json({ status: "ok", rosConnected: rosBridgeService.connected });
});

const PORT = process.env.PORT || 5000;

const start = async () => {
  try {
    await connectDB();

    try {
      rosBridgeService.connect();
    } catch (err) {
      console.error("ROSBridge initial connect failed (will retry):", err.message);
    }

    novncService.start();

    app.listen(PORT, () => {
      console.log(`Server running on port ${PORT}`);
    });
  } catch (err) {
    console.error("Failed to start server:", err.message);
    process.exit(1);
  }
};

start();
