#!/usr/bin/env node

'use strict';

var Service = require('./service');


var co2 = new Service({
  name: 'co2',
  topic: '/sensors/co2',
  op: 'subscribe',
  msgType: 'pandora_sensor_msgs/Co2Msg',
  serverTopic: 'sensors/co2'
}, function(msg) {
  return msg.co2_percentage * 100;
});

co2.start();
