'use strict';

var Backbone = require('backbone');
var ioClient = require('../ros-events');

/**
 * View template.
 */

var qrAlertTemplate = require('../templates/alert.qr.hbs');

/**
 * Alert Model
 */

var Alert = require('../models/alert');


var AlertView = Backbone.View.extend({

  el: '#alert-feed',

  qrTemplate: qrAlertTemplate,

  events: {
    'click .clean-alerts': 'cleanAlerts',
    'click .alert-remove': 'removeAlert',
    'click .alert-info': 'showAlertInfo'
  },

  lastVisualAlert: new Date().getTime() / 1000,
  lastSoundAlert: new Date().getTime() / 1000,
  lastCo2Alert: new Date().getTime() / 1000,
  lastHazmatAlert: new Date().getTime() / 1000,
  lastThermalAlert: new Date().getTime() / 1000,
  lastMotionAlert: new Date().getTime() / 1000,

  visualRate: 5,
  soundRate: 3,
  co2Rate: 3,
  hazmatRate: 3,
  thermalRate: 5,
  motionRate: 4,

  maxAlertRate: 2,

  initialize: function() {
    this.enableAlerts = true;

    var _this = this;
    console.log('View alertfeed initialized');

    $("body").keypress(function(event) {
      if (event.which === 115) {
        console.log('Toggling alerts.');
        console.log(_this.enableAlerts);
        if (_this.enableAlerts === true) {
          _this.enableAlerts = false;
          new PNotify({
            title: 'Alerts disabled',
            hide: true,
            type: 'danger'
          });
        } else {
          new PNotify({
            title: 'Alerts enabled',
            hide: true,
            type: 'success'
          });
          _this.enableAlerts = true;
        }
        console.log(_this.enableAlerts);
      }
    });

    ioClient.on('web/alert/qr', this.appendQRAlert.bind(this));
    ioClient.on('web/alert/hazmat', this.appendHazmatAlert.bind(this));
    ioClient.on('web/alert/motion', this.appendMotionAlert.bind(this));
    ioClient.on('web/alert/co2', this.appendCo2Alert.bind(this));
    ioClient.on('web/alert/visual', this.appendVisualAlert.bind(this));
    ioClient.on('web/alert/thermal', this.appendThermalAlert.bind(this));
    ioClient.on('web/alert/sound', this.appendSoundAlert.bind(this));

  },

  /**
   * Actual victims.
   */

  victims: [],

  /**
   * Signs of life.
   */

  qrs: [],

  appendHazmatAlert: function(msg) {
    if (this.enableAlerts === false) return;
    if (this.dropAlert(this.hazmatRate, this.lastHazmatAlert) === true) return;

    this.lastHazmatAlert = new Date().getTime() / 1000;

    console.log('A Hazmat alert has arrived.');

    new PNotify({
      title: 'Hazmat alert',
      text: 'Pattern type: ' + msg.patternType,
      hide: false,
      type: 'success'
    });
  },

  appendThermalAlert: function(msg) {
    if (this.enableAlerts === false) return;
    if (this.dropAlert(this.thermalRate, this.lastThermalAlert) === true) return;

    this.lastThermalAlert = new Date().getTime() / 1000

    console.log('A Thermal alert has arrived.');

    new PNotify({
      title: 'Thermal alert',
      text: 'Temperature: ' + msg.temperature,
      hide: false,
      type: 'success'
    });
  },

  appendCo2Alert: function(msg) {
    if (this.enableAlerts === false) return;
    if (this.dropAlert(this.co2Rate, this.lastCo2Alert) === true) return;

    this.lastCo2Alert = new Date().getTime() / 1000;

    console.log('A Co2 alert has arrived.');

    new PNotify({
      title: 'Co2 alert',
      text: 'Probability: ' + msg.probability,
      hide: false,
      type: 'success'
    });
  },

  appendMotionAlert: function(msg) {
    if (this.enableAlerts === false) return;
    if (this.dropAlert(this.motionRate, this.lastMotionAlert) === true) return;

    this.lastMotionAlert = new Date().getTime() / 1000;

    console.log('A motion alert has arrived.');

    new PNotify({
      title: 'Motion alert',
      text: 'Probability: ' + msg.probability,
      hide: false,
      type: 'success'
    });
  },

  appendSoundAlert: function(msg) {
    if (this.enableAlerts === false) return;
    if (this.dropAlert(this.soundRate, this.lastSoundAlert) === true) return;
    if (msg.word == 0) return;

    this.lastSoundAlert = new Date().getTime() / 1000;

    console.log('A sound alert has arrived.');

    new PNotify({
      title: 'Sound alert',
      text: 'Word: ' + msg.word + ', Probability: ' + msg.probability,
      hide: false,
      type: 'success'
    });
  },

  appendVisualAlert: function(msg) {
    if (this.enableAlerts === false) return;
    if (this.dropAlert(this.visualRate, this.lastVisualAlert) === true) return;

    this.lastVisualAlert = new Date().getTime() / 1000;

    console.log('A visual alert has arrived.');

    new PNotify({
      title: 'Visual alert',
      text: 'Probability: ' + msg.probability,
      hide: false,
      type: 'success'
    });
  },

  appendQRAlert: function(msg) {

    console.log('A QR alert has arrived.');

    // Create a new alert model for this alert.
    var alert = new Alert(msg);
    alert.set({'type': 'QR'});

    // If a qr already exists.
    for (var i = 0; i < this.qrs.length; i++) {
      if (this.qrs[i].get('content') == msg.content) {
        return;
      }
    }

    // Show stacked notification.
    new PNotify({
      title: 'A QR arrived',
      text: 'Content: ' + msg.content + ', Probability: ' + msg.probability,
      hide: false,
      type: 'success'
    });

    this.qrs.push(alert);

    // Append the alert into the list.
    this.$el.append(this.qrTemplate(alert.toJSON()));

  },

  dropAlert: function(rate, lastAlert) {

    var timeOfCurrentAlert = new Date().getTime() / 1000;
    var timeFromLastAlert = timeOfCurrentAlert - lastAlert

    if (timeFromLastAlert < rate) {
      return true;
    } else {
      return false;
    }

  },

  render: function() {

    this.$el.html(this.qrTemplate());

    return this;
  }

});


module.exports = AlertView;
