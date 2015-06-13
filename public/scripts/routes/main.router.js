'use strict';

var Backbone = require('backbone');

/**
 * Layout Views.
 */

var LayoutManager = require('./layout.manager');

var OverView = require('../views/overview');
var SensorsLayout =  require('../views/layouts/sensors.layout');
var CamerasLayout = require('../views/layouts/cameras.layout');


var Router = Backbone.Router.extend({

  routes: {
    'sensors': 'renderSensors',
    'overview': 'renderOverview',
    'camera': 'renderCameras'
  },

  manager: new LayoutManager({
    layouts: {
    'sensors': new SensorsLayout(),
    'cameras': new CamerasLayout(),
    'overview': new OverView()
    }
  }),

  renderSensors: function() {
    console.log('Rendering sensors');
    this.manager.render('sensors');
  },

  renderOverview: function() {
    this.manager.render('overview');
  },

  renderCameras: function() {
    console.log('Rendering cameras.');
    this.manager.render('cameras');
  }
});

module.exports = Router;
