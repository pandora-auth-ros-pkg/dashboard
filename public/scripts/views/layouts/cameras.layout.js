'use strict';


/**
 * External dependencies.
 */

var Backbone = require('backbone');
var _ = require('underscore');

/**
 * Views and Models.
 */

var CameraView = require('../camera');
var Camera = require('../../models/camera');

/**
 * Templates.
 */

var layoutTemplate = require('../../templates/layouts/cameras.layout.hbs');

var CamerasLayout = Backbone.View.extend({

  el: $('#main-dashboard-content'),

  template: layoutTemplate,

  /**
   * Declare what subviews will be renderd in this layout.
   */

  views: {
    'camera.camera': {
      view: new CameraView({
        model: new Camera({
          name: 'Camera'
        })
      }),
      selector: '#camera-section'
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

module.exports = CamerasLayout;
