const API_BASE = "/api";

async function request(endpoint, options = {}) {
  const token = localStorage.getItem("gg_token");

  let res;
  try {
    res = await fetch(`${API_BASE}${endpoint}`, {
      headers: {
        "Content-Type": "application/json",
        ...(token && { Authorization: `Bearer ${token}` }),
        ...options.headers,
      },
      ...options,
    });
  } catch {
    throw new Error("Network error - server may be offline");
  }

  // Auto-logout on 401 (expired/invalid token)
  if (res.status === 401) {
    localStorage.removeItem("gg_token");
    localStorage.removeItem("gg_user");
    localStorage.removeItem("selectedRobot");
    window.location.href = "/login";
    throw new Error("Session expired - please login again");
  }

  let data;
  try {
    data = await res.json();
  } catch {
    throw new Error(`Server error (${res.status})`);
  }

  if (!res.ok) {
    throw new Error(data.error || "Request failed");
  }

  return data;
}

// Auth
export const authApi = {
  login: (email, password) =>
    request("/auth/login", {
      method: "POST",
      body: JSON.stringify({ email, password }),
    }),
  register: (email, password, name) =>
    request("/auth/register", {
      method: "POST",
      body: JSON.stringify({ email, password, name }),
    }),
};

// Robots
export const robotApi = {
  getAll: () => request("/robots"),
  getById: (id) => request(`/robots/${id}`),
  create: (data) =>
    request("/robots", { method: "POST", body: JSON.stringify(data) }),
  update: (id, data) =>
    request(`/robots/${id}`, { method: "PUT", body: JSON.stringify(data) }),
  delete: (id) => request(`/robots/${id}`, { method: "DELETE" }),
};

// Telemetry
export const telemetryApi = {
  getLatest: (robotId) => request(`/telemetry/${robotId}/latest`),
  getHistory: (robotId, limit = 30) =>
    request(`/telemetry/${robotId}/history?limit=${limit}`),
};
