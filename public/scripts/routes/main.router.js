'use strict';

var Backbone = require('backbone');

/**
 * Layout Views.
 */

var LayoutManager = require('./layout.manager');

var OverView = require('../views/overview');
var SensorsLayout =  require('../views/layouts/sensors.layout');
var CamerasLayout = require('../views/layouts/cameras.layout');
var AgentLayout = require('../views/layouts/agent.layout');
var AlertsLayout = require('../views/layouts/alerts.layout');
var GeotiffLayout = require('../views/layouts/geotiff.layout');
var CSVLayout = require('../views/layouts/csv.layout');


var Router = Backbone.Router.extend({

  routes: {
    'sensors': 'renderSensors',
    'overview': 'renderOverview',
    'camera': 'renderCameras',
    'agent': 'renderAgent',
    'geotiff': 'renderGeotiff',
    'csv': 'renderCSV',
    '*path': 'renderOverview'
  },

  manager: new LayoutManager({
    layouts: {
      'sensors': new SensorsLayout(),
      'cameras': new CamerasLayout(),
      'overview': new OverView(),
      'agent': new AgentLayout(),
      'geotiff': new GeotiffLayout(),
      'csv': new CSVLayout(),
      'alerts': new AlertsLayout()
    }
  }),

  renderSensors: function() {
    console.log('Rendering sensors');
    this.manager.render('sensors');
  },

  renderOverview: function() {
    this.manager.render('overview');
    if (!this.manager.isRendered('alerts')) {
      console.log('Rendering alerts.');
      this.manager.render('alerts');
    } else {
      console.log('Alerts layout is already rendered.');
    }
  },

  renderAgent: function() {
    this.manager.render('agent');
  },

  renderCameras: function() {
    console.log('Rendering cameras.');
    this.manager.render('cameras');
  },

  renderGeotiff: function() {
    console.log('Rendering geotiff.');
    this.manager.render('geotiff');
  },

  renderCSV: function() {
    console.log('Rendering csv.');
    this.manager.render('csv');
  }
});

module.exports = Router;
