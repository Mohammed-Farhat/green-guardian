const express = require("express");
const router = express.Router();
const auth = require("../middleware/auth");
const {
  getLatestTelemetry,
  getTelemetryHistory,
  getAllLatestTelemetry,
} = require("../controllers/telemetryController");

router.get("/all/latest", auth, getAllLatestTelemetry);
router.get("/:robotId/latest", auth, getLatestTelemetry);
router.get("/:robotId/history", auth, getTelemetryHistory);

module.exports = router;
