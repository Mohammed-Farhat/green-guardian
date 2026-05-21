const API_BASE = "/api";

function getAuthHeaders() {
  const token = localStorage.getItem("token");
  return token ? { Authorization: `Bearer ${token}` } : {};
}

async function request(endpoint, options = {}) {
  let res;
  try {
    res = await fetch(`${API_BASE}${endpoint}`, {
      headers: {
        "Content-Type": "application/json",
        ...getAuthHeaders(),
        ...options.headers,
      },
      ...options,
    });
  } catch {
    throw new Error("Network error - server may be offline");
  }

  if (res.status === 401) {
    localStorage.removeItem("token");
    window.location.href = "/login";
    throw new Error("Session expired");
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

export const telemetryApi = {
  getLatest: () => request("/telemetry/latest"),
  getHistory: (limit = 30) => request(`/telemetry/history?limit=${limit}`),
};

export const robotApi = {
  emptyBins: () => request("/robot/empty-bins", { method: "POST" }),
  shutdown: () => request("/robot/shutdown", { method: "POST" }),
  teleop: (linear, angular) =>
    request("/robot/teleop", {
      method: "POST",
      body: JSON.stringify({ linear, angular }),
    }),
  stop: () => request("/robot/stop", { method: "POST" }),
};
