const ROSLIB = require("roslib");
const RobotTelemetry = require("../models/RobotTelemetry");
const Robot = require("../models/Robot");

class RosBridgeService {
  constructor() {
    this.ros = null;
    this.subscribers = {};
    this.latestData = {}; // { robotId: { speed, temperature, ... } }
    this.saveInterval = null;
    this.connected = false;
  }

  connect() {
    const url = process.env.ROSBRIDGE_URL || "ws://localhost:9090";

    this.ros = new ROSLIB.Ros({ url });

    this.ros.on("connection", () => {
      console.log(`Connected to ROSBridge at ${url}`);
      this.connected = true;
      this._subscribeAllRobots();
    });

    this.ros.on("error", (error) => {
      console.error("ROSBridge error:", error.message || error);
    });

    this.ros.on("close", () => {
      console.log("ROSBridge connection closed. Reconnecting in 5s...");
      this.connected = false;
      setTimeout(() => this.connect(), 5000);
    });

    this._startSaveInterval();
  }

  async _subscribeAllRobots() {
    try {
      const robots = await Robot.find({ status: "online" });
      for (const robot of robots) {
        this.subscribeToRobot(robot);
      }
    } catch (err) {
      console.error("Error fetching robots for subscription:", err.message);
    }
  }

  subscribeToRobot(robot) {
    const robotId = robot._id.toString();
    const topics = robot.rosTopics;

    if (this.subscribers[robotId]) return;

    this.latestData[robotId] = {
      speed: 0,
      temperature: 0,
      humidity: 0,
      battery: 0,
      binOrganic: 0,
      binNonOrganic: 0,
      cpuTemp: 0,
      fanSpeed: 0,
      gps: { lat: 0, lng: 0 },
    };

    this.subscribers[robotId] = [];

    // Speed from /odom (nav_msgs/Odometry)
    this._subscribe(robotId, topics.speed, "nav_msgs/Odometry", (msg) => {
      const vx = msg.twist.twist.linear.x;
      const vy = msg.twist.twist.linear.y;
      this.latestData[robotId].speed = Math.sqrt(vx * vx + vy * vy);
    });

    // Temperature (std_msgs/Float64)
    this._subscribe(robotId, topics.temperature, "std_msgs/Float64", (msg) => {
      this.latestData[robotId].temperature = msg.data;
    });

    // Humidity (std_msgs/Float64)
    this._subscribe(robotId, topics.humidity, "std_msgs/Float64", (msg) => {
      this.latestData[robotId].humidity = msg.data;
    });

    // GPS (sensor_msgs/NavSatFix)
    this._subscribe(robotId, topics.gps, "sensor_msgs/NavSatFix", (msg) => {
      this.latestData[robotId].gps.lat = msg.latitude;
      this.latestData[robotId].gps.lng = msg.longitude;
    });

    // Battery (std_msgs/Float64)
    this._subscribe(robotId, topics.battery, "std_msgs/Float64", (msg) => {
      this.latestData[robotId].battery = msg.data;
    });

    // Bin status (custom: { organic: float, nonOrganic: float })
    this._subscribe(robotId, topics.binStatus, "std_msgs/String", (msg) => {
      try {
        const data = JSON.parse(msg.data);
        this.latestData[robotId].binOrganic = data.organic || 0;
        this.latestData[robotId].binNonOrganic = data.nonOrganic || 0;
      } catch {
        // ignore parse errors
      }
    });

    // CPU Temp (std_msgs/Float64)
    this._subscribe(robotId, topics.cpuTemp, "std_msgs/Float64", (msg) => {
      this.latestData[robotId].cpuTemp = msg.data;
    });

    // Fan Speed (std_msgs/Float64)
    this._subscribe(robotId, topics.fanSpeed, "std_msgs/Float64", (msg) => {
      this.latestData[robotId].fanSpeed = msg.data;
    });

    console.log(`Subscribed to ROS topics for robot: ${robot.name}`);
  }

  _subscribe(robotId, topic, messageType, callback) {
    if (!this.ros || !this.connected) return;

    const listener = new ROSLIB.Topic({
      ros: this.ros,
      name: topic,
      messageType,
    });

    listener.subscribe(callback);
    this.subscribers[robotId].push(listener);
  }

  unsubscribeRobot(robotId) {
    const subs = this.subscribers[robotId];
    if (subs) {
      subs.forEach((listener) => listener.unsubscribe());
      delete this.subscribers[robotId];
      delete this.latestData[robotId];
    }
  }

  _startSaveInterval() {
    const interval = parseInt(process.env.SAVE_INTERVAL_MS) || 10000;

    this.saveInterval = setInterval(async () => {
      await this._saveAllTelemetry();
    }, interval);

    console.log(`Telemetry save interval: every ${interval / 1000}s`);
  }

  async _saveAllTelemetry() {
    const robotIds = Object.keys(this.latestData);
    if (robotIds.length === 0) return;

    const docs = robotIds.map((robotId) => ({
      robotId,
      ...this.latestData[robotId],
      timestamp: new Date(),
    }));

    try {
      await RobotTelemetry.insertMany(docs);
      console.log(`Saved telemetry for ${docs.length} robot(s)`);

      // Update lastSeen for all robots
      await Robot.updateMany(
        { _id: { $in: robotIds } },
        { lastSeen: new Date() }
      );
    } catch (err) {
      console.error("Error saving telemetry:", err.message);
    }
  }

  getLatestData(robotId) {
    return this.latestData[robotId] || null;
  }

  getAllLatestData() {
    return this.latestData;
  }

  disconnect() {
    if (this.saveInterval) clearInterval(this.saveInterval);
    Object.keys(this.subscribers).forEach((id) => this.unsubscribeRobot(id));
    if (this.ros) this.ros.close();
  }
}

// Singleton
const rosBridgeService = new RosBridgeService();
module.exports = rosBridgeService;
