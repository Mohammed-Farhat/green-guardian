const ROSLIB = require("roslib");
const RobotTelemetry = require("../models/RobotTelemetry");

class RosBridgeService {
  constructor() {
    this.ros = null;
    this.subscribers = [];
    this.latestData = {
      speed: 0,
      cpuTemp: 0,
      cpuUsage: 0,
      ramUsage: 0,
      fanSpeed: 0,
      binOrganic: 0,
      binNonOrganic: 0,
      battery: 0,
    };
    this.saveInterval = null;
    this.connected = false;
  }

  connect() {
    const url = process.env.ROSBRIDGE_URL || "ws://192.168.0.94:9090";

    this.ros = new ROSLIB.Ros({ url });

    this.ros.on("connection", () => {
      console.log(`Connected to ROSBridge at ${url}`);
      this.connected = true;
      this._subscribeToTopics();
    });

    this.ros.on("error", (error) => {
      console.error("ROSBridge error:", error.message || error);
    });

    this.ros.on("close", () => {
      console.log("ROSBridge connection closed. Reconnecting in 5s...");
      this.connected = false;
      this.subscribers = [];
      setTimeout(() => this.connect(), 5000);
    });

    this._startSaveInterval();
  }

  _subscribeToTopics() {
    // Speed from /odom (nav_msgs/Odometry)
    this._subscribe("/odom", "nav_msgs/Odometry", (msg) => {
      const vx = msg.twist.twist.linear.x;
      const vy = msg.twist.twist.linear.y;
      this.latestData.speed = Math.sqrt(vx * vx + vy * vy);
    });

    // CPU Temperature (std_msgs/Float32)
    this._subscribe("/system/cpu_temp", "std_msgs/Float32", (msg) => {
      this.latestData.cpuTemp = msg.data;
    });

    // CPU Usage (std_msgs/Float32)
    this._subscribe("/system/cpu_usage", "std_msgs/Float32", (msg) => {
      this.latestData.cpuUsage = msg.data;
    });

    // RAM Usage (std_msgs/Float32)
    this._subscribe("/system/ram_usage", "std_msgs/Float32", (msg) => {
      this.latestData.ramUsage = msg.data;
    });

    // Fan Speed (std_msgs/Float32)
    this._subscribe("/system/fan_speed", "std_msgs/Float32", (msg) => {
      this.latestData.fanSpeed = msg.data;
    });

    // Organic Bin Level (std_msgs/Float32)
    this._subscribe("/bins/organic_level", "std_msgs/Float32", (msg) => {
      this.latestData.binOrganic = msg.data;
    });

    // Non-Organic Bin Level (std_msgs/Float32)
    this._subscribe("/bins/non_organic_level", "std_msgs/Float32", (msg) => {
      this.latestData.binNonOrganic = msg.data;
    });

    console.log("Subscribed to all ROS2 topics");
  }

  _subscribe(topic, messageType, callback) {
    if (!this.ros || !this.connected) return;

    const listener = new ROSLIB.Topic({
      ros: this.ros,
      name: topic,
      messageType,
    });

    listener.subscribe(callback);
    this.subscribers.push(listener);
  }

  // Call /bins/reset service
  callEmptyBins() {
    return this._callService("/bins/reset", "std_srvs/Trigger");
  }

  // Call /robot/shutdown service
  callShutdown() {
    return this._callService("/robot/shutdown", "std_srvs/Trigger");
  }

  _callService(name, type) {
    return new Promise((resolve, reject) => {
      if (!this.ros || !this.connected) {
        return reject(new Error("Not connected to ROSBridge"));
      }

      const service = new ROSLIB.Service({
        ros: this.ros,
        name,
        serviceType: type,
      });

      service.callService(
        new ROSLIB.ServiceRequest({}),
        resolve,
        (err) => reject(new Error(typeof err === "string" ? err : "Service call failed"))
      );
    });
  }

  _startSaveInterval() {
    const interval = parseInt(process.env.SAVE_INTERVAL_MS) || 10000;

    this.saveInterval = setInterval(async () => {
      await this._saveTelemetry();
    }, interval);

    console.log(`Telemetry save interval: every ${interval / 1000}s`);
  }

  async _saveTelemetry() {
    try {
      await RobotTelemetry.create({
        ...this.latestData,
        timestamp: new Date(),
      });
    } catch (err) {
      console.error("Error saving telemetry:", err.message);
    }
  }

  getLatestData() {
    return this.latestData;
  }

  disconnect() {
    if (this.saveInterval) clearInterval(this.saveInterval);
    this.subscribers.forEach((listener) => listener.unsubscribe());
    this.subscribers = [];
    if (this.ros) this.ros.close();
  }
}

const rosBridgeService = new RosBridgeService();
module.exports = rosBridgeService;
