const API_BASE = "/api";

async function request(endpoint, options = {}) {
  let res;
  try {
    res = await fetch(`${API_BASE}${endpoint}`, {
      headers: {
        "Content-Type": "application/json",
        ...options.headers,
      },
      ...options,
    });
  } catch {
    throw new Error("Network error - server may be offline");
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
};
