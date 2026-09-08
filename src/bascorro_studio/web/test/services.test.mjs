import assert from "node:assert/strict";
import { EventEmitter } from "node:events";
import test from "node:test";
import { callRosService, waitForRobot } from "../src/net/services.js";

function fakeService(reply) {
  const ros = new EventEmitter();
  ros.isConnected = true;
  ros.idCounter = 0;
  ros.sent = [];
  ros.callOnConnection = (request) => {
    ros.sent.push(request);
    if (reply) queueMicrotask(() => ros.emit(request.id, reply));
  };
  return { ros, name: "/test/get_parameters", serviceType: "rcl_interfaces/srv/GetParameters" };
}

test("service success returns actual data and cleans up every listener", async () => {
  const service = fakeService({ result: true, values: { parameters: { period_time: 0.8 } } });
  assert.deepEqual(await callRosService(service, { get_param: true }), { parameters: { period_time: 0.8 } });
  assert.equal(service.ros.sent[0].op, "call_service");
  assert.deepEqual(service.ros.eventNames(), []);
});

test("service failure handles rosbridge string errors", async () => {
  const service = fakeService({ result: false, values: "service unavailable" });
  await assert.rejects(callRosService(service), /service unavailable/);
  assert.deepEqual(service.ros.eventNames(), []);
});

test("missing services time out instead of leaving the UI permanently busy", async () => {
  const service = fakeService();
  await assert.rejects(callRosService(service, {}, { timeoutMs: 10 }), /tidak menjawab/);
  assert.deepEqual(service.ros.eventNames(), []);
});

test("disconnected requests are not queued to run on reconnection", async () => {
  const service = fakeService();
  service.ros.isConnected = false;
  await assert.rejects(callRosService(service));
  assert.equal(service.ros.sent.length, 0);
});

test("disconnect and cancellation invalidate pending responses", async () => {
  for (const event of ["close", "error"]) {
    const service = fakeService();
    const result = callRosService(service);
    service.ros.emit(event);
    await assert.rejects(result, /terputus/);
    assert.deepEqual(service.ros.eventNames(), []);
  }
  const service = fakeService();
  const controller = new AbortController();
  const result = callRosService(service, {}, { signal: controller.signal });
  controller.abort();
  await assert.rejects(result, /dibatalkan/);
  assert.deepEqual(service.ros.eventNames(), []);
});

test("Stop cancels a pending settle wait before Start can execute", async () => {
  const controller = new AbortController();
  let started = false;
  const sequence = waitForRobot(100, controller.signal).then(() => { started = true; });
  controller.abort();
  await assert.rejects(sequence, /dibatalkan/);
  assert.equal(started, false);
});
