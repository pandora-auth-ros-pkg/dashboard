'use strict';

/**
 * Dependencies
 */

var Backbone = require('backbone');
var Socket = require('../ros-events.js');

/**
 * Handlebars templates
 */

var overviewTemplate = require('../templates/overview.hbs');
var tableItem = require('../templates/table-item.hbs');


var OverView = Backbone.View.extend({

  el: '#main-dashboard-content',

  nodesTable: '#nodes tbody',
  topicsTable: '#topics tbody',
  servicesTable: '#services tbody',

  overviewTemplate: overviewTemplate,
  tableItemTemplate: tableItem,

  nodes: [],
  topics: [],
  services: [],

  events: {
    'click #nodes-tab': 'getROSNodes',
    'click #topics-tab': 'getROSTopics',
    'click #services-tab': 'getROSServices'
  },

  initialize: function() {
    console.log('View initialized');

    Socket.on('web/rosapi/nodes', this.renderROSNodes.bind(this));
    Socket.on('web/rosapi/topics', this.renderROSTopics.bind(this));
    Socket.on('web/rosapi/services', this.renderROSServices.bind(this));
  },

  getROSNodes: function() {
    Socket.emit('web/rosapi/request', 'nodes');
  },

  renderROSNodes: function(nodes) {
    this.nodes = nodes;
    this.$(this.nodesTable).html(this.tableItemTemplate({"items": this.nodes}));
  },

  getROSTopics: function(msg) {
    Socket.emit('web/rosapi/request', 'topics');
  },

  renderROSTopics: function(topics) {
    this.topics = topics;
    this.$(this.topicsTable).html(this.tableItemTemplate(
      {"items": this.topics}
    ));
  },

  getROSServices: function() {
    console.log('Listing ROS services.');
    Socket.emit('web/rosapi/request', 'services');
  },

  renderROSServices: function(services) {
    this.services = services;
    this.$(this.servicesTable).html(this.tableItemTemplate(
      {"items": this.services}
    ));
  },

  render: function() {
    this.$el.html(this.overviewTemplate());
    this.getROSNodes();
    return this;
  }

});

module.exports = OverView;
