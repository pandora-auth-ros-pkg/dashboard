#!/usr/bin/env node

'use strict';

var Service = require('./service');


var battery = new Service({
  name: 'batteries',
  topic: '/sensors/battery',
  op: 'subscribe',
  msgType: 'pandora_sensor_msgs/BatteryMsg',
  serverTopic: 'sensors/battery'
}, function(msg) {
  return msg;
});

battery.start();
