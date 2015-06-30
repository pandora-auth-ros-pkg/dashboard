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


var Router = Backbone.Router.extend({

  routes: {
    'sensors': 'renderSensors',
    'overview': 'renderOverview',
    'camera': 'renderCameras',
    'agent': 'renderAgent',
    '*path': 'renderOverview'
  },

  manager: new LayoutManager({
    layouts: {
    'sensors': new SensorsLayout(),
    'cameras': new CamerasLayout(),
    'overview': new OverView(),
    'agent': new AgentLayout(),
    'alerts': new AlertsLayout()
    }
  }),

  renderSensors: function() {
    console.log('Rendering sensors');
    this.manager.render('sensors');
  },

  renderOverview: function() {
    this.manager.render('overview');
    this.manager.render('alerts');
  },

  renderAgent: function() {
    this.manager.render('agent');
  },

  renderCameras: function() {
    console.log('Rendering cameras.');
    this.manager.render('cameras');
  }
});

module.exports = Router;
