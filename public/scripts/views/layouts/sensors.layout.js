'use strict';


/**
 * External dependencies.
 */

var Backbone = require('backbone');
var _ = require('underscore');

/**
 * Views and Models.
 */

var Battery = require('../../models/battery');
var Temperature = require('../../models/temperature');
var SimpleSensor = require('../../models/simple.sensor');
var IMU = require('../../models/imu');
var Sonar = require('../../models/sonar');

var BatteryView = require('../battery');
var TemperatureView = require('../temperature');
var SimpleSensorView = require('../simple.sensor');
var IMUView = require('../imu');
var SonarView = require('../sonar');

/**
 * Templates.
 */

var layoutTemplate = require('../../templates/layouts/sensors.layout.hbs');

var SensorsLayout = Backbone.View.extend({

  el: $('#main-dashboard-content'),

  template: layoutTemplate,

  /**
   * Declare what subviews will be renderd in this layout.
   */

  views: {
    'battery.psu': {
      view: new BatteryView({
        model: new Battery({
          name: 'PSU',
          topic: 'web/sensors/battery/psu'
        })
      }),
      selector: '#batteries #psu'
    },
    'battery.motors': {
      view: new BatteryView({
        model: new Battery({
          name: 'Motors',
          topic: 'web/sensors/battery/motors'
        })
      }),
      selector: '#batteries #motors'
    },
    'cpu.temperatures': {
      view: new TemperatureView({
        model: new Temperature({
          name: 'Cpu Temperature',
          topic: 'web/sensors/temperature'
        })
      }),
      selector: '#temperature'
    },
    'co2.simple.sensor': {
      view: new SimpleSensorView({
        model: new SimpleSensor({
          name: 'CO2',
          icon: 'rss',
          unit: '%',
          topic: 'web/sensors/co2'
        })
      }),
      selector: '.simple-sensors #co2'
    },
    'thermal.simple.sensor': {
      view: new SimpleSensorView({
        model: new SimpleSensor({
          name: 'Thermal mean',
          icon: 'fire',
          unit: '°C',
          topic: 'web/sensors/thermal'
        })
      }),
      selector: '.simple-sensors #thermal'
    },
    'imu': {
      view: new IMUView({
        model: new IMU({
          name: 'IMU measurements',
          topic: 'web/sensors/imu'
        })
      }),
      selector: '#imu'
    },
    'sonar': {
      view: new SonarView({
        model: new Sonar({
          name: 'Sonar',
          topic: 'web/sensors/sonar'
        })
      }),
      selector: '#sonars'
    }
  },

  initialize: function() {
  },

  /**
   * Change the context of every sub view into the
   * main layout.
   */

  placeInContext: function() {
    var _this = this;
    _.each(this.views, function(item) {
      item.view.$el = _this.$(item.selector);
    });
  },

  /**
   * Render every subview on the layout.
   */

  renderSubViews: function() {
    _.each(this.views, function(item) {
      item.view.render();
      item.view.delegateEvents();
    });
  },

  render: function() {
    this.$el.html(this.template());
    this.placeInContext();
    this.renderSubViews();
    return this;
  }

});

module.exports = SensorsLayout;
