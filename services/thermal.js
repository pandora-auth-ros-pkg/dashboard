#!/usr/bin/env node

'use strict';

var Service = require('./service');


var thermal = new Service({
  name: 'thermal',
  topic: '/sensors/thermal',
  op: 'subscribe',
  msgType: 'pandora_sensor_msgs/ThermalMeanMsg',
  serverTopic: 'sensors/thermal'
}, function(msg) {
  return msg.thermal_mean;
});

thermal.start();
