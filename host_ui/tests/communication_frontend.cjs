const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");
const path = require("node:path");

const scriptPath = path.join(__dirname, "..", "web", "app.js");
const source = fs.readFileSync(scriptPath, "utf8").replace(
  '  if (document.readyState === "loading") {',
  `  globalThis.communication = {state, connectWs, wsSend, tickConnectionHealth, authHeaders};
  if (document.readyState === "loading") {`
);

function harness() {
  let now = 100000;
  let timerId = 0;
  const timers = new Map();
  const sockets = [];
  class FakeWebSocket {
    static CONNECTING = 0;
    static OPEN = 1;
    constructor() {
      this.readyState = 0;
      this.bufferedAmount = 0;
      this.sent = [];
      sockets.push(this);
    }
    send(message) {
      if (this.failSend) throw new Error("send failed");
      this.sent.push(message);
    }
    close(code) { this.closed = code || 1000; }
  }
  const element = {textContent: "", classList: {toggle() {}, remove() {}}};
  const context = vm.createContext({
    document: {
      readyState: "loading", getElementById: () => element,
      querySelectorAll: () => [], addEventListener() {},
    },
    MapView: class {},
    WebSocket: FakeWebSocket,
    location: {protocol: "http:", host: "localhost"},
    Date: {now: () => now},
    performance: {now: () => now},
    setTimeout: (callback) => { timers.set(++timerId, callback); return timerId; },
    clearTimeout: (id) => timers.delete(id),
    setInterval: () => ++timerId, clearInterval() {},
  });
  vm.runInContext(source, context);
  return {
    ...context.communication, sockets, timers,
    advance: (ms) => { now += ms; },
    open() {
      context.communication.connectWs();
      const socket = sockets.at(-1);
      socket.readyState = 1;
      socket.onopen();
      return socket;
    },
  };
}

{
  const h = harness();
  h.connectWs();
  h.advance(10001);
  h.tickConnectionHealth();
  assert.equal(h.state.ws, null);
  assert.equal(h.state.connected, false);
  assert.equal(h.timers.size, 1);
}

{
  const h = harness();
  h.open();
  h.state.keys.add("w");
  h.state.sessionToken = "retired";
  h.state.wsCommandQueue.push({type: "arm"});
  h.advance(5001);
  h.tickConnectionHealth();
  assert.equal(h.state.connected, false);
  assert.equal(h.state.keys.size, 0);
  assert.equal(h.state.wsCommandQueue.length, 0);
  assert.equal(h.state.sessionToken, null);
  assert.equal(h.wsSend({type: "teleop"}), false);
  assert.equal(h.timers.size, 1);
}

for (const mode of ["buffered", "throw"]) {
  const h = harness();
  const socket = h.open();
  if (mode === "buffered") socket.bufferedAmount = 65537;
  else socket.failSend = true;
  assert.equal(h.wsSend({type: "teleop"}), false);
  assert.equal(socket.sent.length, 0);
  assert.equal(h.state.ws, null);
  assert.equal(h.timers.size, 1);
}

{
  const h = harness();
  const old = h.open();
  const oldClose = old.onclose;
  const current = h.open();
  oldClose({code: 1006});
  old.onmessage({data: '{"type":"hello","data":{}}'});
  assert.equal(h.state.ws, current);
  assert.equal(h.state.connected, true);
  h.state.sessionToken = "current-session";
  assert.equal(h.authHeaders()["X-Control-Session"], "current-session");
  current.onclose({code: 4001});
  assert.equal(h.state.connected, false);
  assert.equal(h.timers.size, 0);
}

console.log("Frontend communication regression checks passed");
