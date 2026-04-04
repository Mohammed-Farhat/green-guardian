require("dotenv").config();
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

// Middleware
app.use(
  cors({
    origin: process.env.ALLOWED_ORIGINS
      ? process.env.ALLOWED_ORIGINS.split(",")
      : "*",
  })
);
app.use(express.json());

// Routes
app.use("/api/auth", authRoutes);
app.use("/api/telemetry", telemetryRoutes);
app.use("/api/robot", auth, robotRoutes);

// Health check
app.get("/api/health", (req, res) => {
  res.json({ status: "ok" });
});

const PORT = process.env.PORT || 5000;

const start = async () => {
  try {
    await connectDB();
    rosBridgeService.connect();
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
