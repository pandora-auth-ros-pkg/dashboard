#!/usr/bin/env node

'use strict';

var Service = require('./service');

var qrTopic = '/data_fusion/qr_info';

var hazmatTopic = '/vision/hazmat_alert';
var hazmatMsg = 'pandora_vision_msgs/HazmatAlertVector';

var thermalTopic = '/vision/thermal_direction_alert';
var thermalMsg = 'pandora_vision_msgs/ThermalAlertVector';

var soundTopic = '/sound/complete_alert';
var soundMsg = 'pandora_audio_msgs/SoundAlertVector';

var motionTopic = '/vision/motion_alert';
var visualTopic = '/vision/victim_direction_alert';
var co2Topic = '/sensor_processing/co2_alert';

var generalMsg = 'pandora_common_msgs/GeneralAlertVector';


var QRAlert = new Service({
  name: 'qrAlert',
  topic: qrTopic,
  op: 'subscribe',
  msgType: 'pandora_data_fusion_msgs/QrInfo',
  serverTopic: 'alert/qr'
}, function(msg) {
  var date = new Date(msg.timeFound.secs * 1000);

  return {
    id: msg.id,
    content: msg.content,
    probability: msg.probability.toFixed(2),
    timeFound: date.toString(),
    frameID: msg.qrPose.header.frame_id,
    qrFrameId: msg.qrFrameId
  };
});

var hazmatAlert  = new Service({
  name: 'hazmatAlert',
  topic: hazmatTopic,
  op: 'subscribe',
  msgType: hazmatMsg,
  serverTopic: 'alert/hazmat'
}, function(msg) {

  var alert = msg.alerts[0];

  return {
    patternType: alert.patternType,
    probability: alert.info.probability.toFixed(2)
  };
});


var thermalAlert = new Service({
  name: 'thermalAlert',
  topic: thermalTopic,
  op: 'subscribe',
  msgType: thermalMsg,
  serverTopic: 'alert/thermal'
}, function(msg) {
  var alert = msg.alerts[0];

  return {
    temperature: alert.temperature.toFixed(2),
    probability: alert.info.probability.toFixed(2)
  };
});

var soundAlert  = new Service({
  name: 'soundAlert ',
  topic: soundTopic,
  op: 'subscribe',
  msgType: soundMsg,
  serverTopic: 'alert/sound'
}, function(msg) {
  var alert = msg.alerts[0];

  return {
    word: alert.word,
    probability: alert.info.probability.toFixed(2)
  };
});

var co2Alert = new Service({
  name: 'co2Alert ',
  topic: co2Topic,
  op: 'subscribe',
  msgType: generalMsg,
  serverTopic: 'alert/co2'
}, function(msg) {
  var alert = msg.alerts[0];

  return {
    probability: alert.probability.toFixed(2)
  };
});


var motionAlert  = new Service({
  name: 'motionAlert ',
  topic: motionTopic,
  op: 'subscribe',
  msgType: generalMsg,
  serverTopic: 'alert/motion'
}, function(msg) {
  var alert = msg.alerts[0];

  return {
    probability: alert.probability.toFixed(2)
  };
});


var visualAlert  = new Service({
  name: 'visualAlert ',
  topic: visualTopic,
  op: 'subscribe',
  msgType: generalMsg,
  serverTopic: 'alert/visual'
}, function(msg) {
  var alert = msg.alerts[0];

  return {
    probability: alert.probability.toFixed(2)
  };
});

QRAlert.start();
hazmatAlert.start();
soundAlert.start();
thermalAlert.start();
co2Alert.start();
motionAlert.start();
visualAlert.start();
