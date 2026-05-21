const express = require("express");
const router = express.Router();
const {
  getLatestTelemetry,
  getTelemetryHistory,
} = require("../controllers/telemetryController");

router.get("/latest", getLatestTelemetry);
router.get("/history", getTelemetryHistory);

module.exports = router;
