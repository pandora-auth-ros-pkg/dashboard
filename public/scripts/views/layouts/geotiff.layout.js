'use strict';


/**
 * External dependencies.
 */

var Backbone = require('backbone');
var _ = require('underscore');

/**
 * Views and Models.
 */

var GeotiffView = require('../geotiff');

/**
 * Templates.
 */

var layoutTemplate = require('../../templates/layouts/geotiff.layout.hbs');

var GeotiffLayout = Backbone.View.extend({

  el: $('#main-dashboard-content'),

  template: layoutTemplate,

  /**
   * Declare what subviews will be renderd in this layout.
   */

  views: {
    'geotiff': {
      view: new GeotiffView({}),
      selector: '#geotiff-section'
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

module.exports = GeotiffLayout;
