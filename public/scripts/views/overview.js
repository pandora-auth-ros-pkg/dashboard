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
var modalInfo = require('../templates/modal-topic-info.hbs');


var OverView = Backbone.View.extend({

  el: '#main-dashboard-content',

  nodesTable: '#nodes tbody',
  topicsTable: '#topics tbody',
  servicesTable: '#services tbody',

  overviewTemplate: overviewTemplate,
  tableItemTemplate: tableItem,
  modalInfoTemplate: modalInfo,

  nodes: [],
  topics: [],
  services: [],

  waitingForService: false,

  events: {
    'click #nodes-tab': 'getROSNodes',
    'click #topics-tab': 'getROSTopics',
    'click #services-tab': 'getROSServices',

    'click .publishers-button': 'getTopicPublishers',
    'click .subscribers-button': 'getTopicSubscribers'
  },

  initialize: function() {
    console.log('View initialized');

    Socket.on('web/rosapi/nodes', this.renderROSNodes.bind(this));
    Socket.on('web/rosapi/topics', this.renderROSTopics.bind(this));
    Socket.on('web/rosapi/services', this.renderROSServices.bind(this));

    Socket.on('web/rosapi/subscribers/response', this.showSubs.bind(this));
    Socket.on('web/rosapi/publishers/response', this.showPubs.bind(this));
  },

  showPubs: function(pubs) {
    if (!this.waitingForService) return;
    if (pubs.length === 0) return;

    console.log('Show modal.');
    $('#topic-info-modal .modal-body').html(this.modalInfoTemplate({
      "items": pubs
    }));
    $('#topic-info-modal').modal('show');
  },

  showSubs: function(subs) {
    if (!this.waitingForService) return;

    if (subs.length === 0) {
      new PNotify({
        text: 'No subscribers for this topic.',
        type: 'info',
        animate_speed: 'fast'
      });
      return;
    }

    console.log('Show modal.');
    $('#topic-info-modal .modal-body').html(this.modalInfoTemplate({
      "items": subs
    }));
    $('#topic-info-modal').modal('show');
  },

  getTopicPublishers: function(event) {
    var topic = event.originalEvent.target.id.split('-').pop();
    this.waitingForService = true;
    Socket.emit('web/rosapi/publishers', topic);
  },

  getTopicSubscribers: function(event) {
    var topic = event.originalEvent.target.id.split('-').pop();
    this.waitingForService = true;
    Socket.emit('web/rosapi/subscribers', topic);
  },

  getROSNodes: function() {
    Socket.emit('web/rosapi/request', 'nodes');
  },

  renderROSNodes: function(nodes) {
    this.nodes = nodes.sort();
    this.$(this.nodesTable).html(this.tableItemTemplate({"items": this.nodes}));
  },

  getROSTopics: function(msg) {
    Socket.emit('web/rosapi/request', 'topics');
  },

  renderROSTopics: function(topics) {
    this.topics = topics.sort();
    this.$(this.topicsTable).html(this.tableItemTemplate(
      {
        "items": this.topics,
        "topics": true
      }
    ));
  },

  getROSServices: function() {
    console.log('Listing ROS services.');
    Socket.emit('web/rosapi/request', 'services');
  },

  renderROSServices: function(services) {
    this.services = services.sort();
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
