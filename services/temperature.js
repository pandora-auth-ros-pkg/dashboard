#!/usr/bin/env node

'use strict';

var Service = require('./service');


var co2 = new Service({
  name: 'CPU temp',
  topic: '/cpu/temperature',
  op: 'subscribe',
  msgType: 'pandora_sensor_msgs/Temperature',
  serverTopic: 'sensors/temperature'
}, function(msg) {
  return msg.temperature;
});

co2.start();
