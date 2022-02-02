#!/usr/bin/python
# -*- coding: utf-8 -*-
import math
import sys, re, json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import pylab
from scipy.stats import scoreatpercentile

# @author: Aaron Blankstein, with modifications by Malte Schwarzkopf
class boxplotter(object):
    def __init__(self, median, top, bottom, whisk_top=None,
                 whisk_bottom=None, extreme_top=None):
        self.median = median
        self.top = top
        self.bott = bottom
        self.whisk_top = whisk_top
        self.whisk_bott = whisk_bottom
        self.extreme_top = extreme_top
    def draw_on(self, ax, index, box_color = "blue",
                median_color = "red", whisker_color = "black",
                blw=0.5, mlw=1.0, width=0.7):
        w2 = width / 2
        ax.broken_barh([(index - w2, width)],
                       (self.bott,self.top - self.bott),
                        facecolor="white", edgecolor=box_color, lw=blw)
        ax.broken_barh([(index - w2, width)],
                        (self.median,0),
                        facecolor="white", edgecolor=median_color, lw=mlw)
        if self.whisk_top is not None:
            ax.broken_barh([(index - w2, width)],
                           (self.whisk_top,0),
                            facecolor="white", edgecolor=whisker_color, lw=blw)
            ax.broken_barh([(index , 0)],
                           (self.whisk_top, self.top-self.whisk_top),
                            edgecolor=box_color,linestyle="dashed", lw=blw)
        if self.whisk_bott is not None:
            ax.broken_barh([(index - w2, width)],
                           (self.whisk_bott,0),
                            facecolor="white", edgecolor=whisker_color, lw=blw)
            ax.broken_barh([(index , 0)],
                           (self.whisk_bott,self.bott-self.whisk_bott),
                            edgecolor=box_color, linestyle="dashed", lw=blw)
        if self.extreme_top is not None:
            ax.scatter([index], [self.extreme_top], marker='*',
                        edgecolor=box_color, facecolor=box_color, lw=blw)

def percentile_box_plot(ax, data, indexer=None, index_base=1, index_step=1,
                        box_top=75, box_bottom=25, whisker_top=99,
                        whisker_bottom=1, color='k', label="",
                        box_lw=0.5, median_lw=1.0):
    if indexer is None:
        index_end = index_base + index_step * len(data) + 1
        indexed_data = zip(range(index_base, index_end, index_step), data)
    else:
        indexed_data = [(indexer(datum), datum) for datum in data]
    def get_whisk(vector, w):
        if w is None:
            return None
        return scoreatpercentile(vector, w)

    for index, x in indexed_data:
        if type(color) is list:
            colour = color[index]
        else:
            colour = color
        bp = boxplotter(scoreatpercentile(x, 50),
                        scoreatpercentile(x, box_top),
                        scoreatpercentile(x, box_bottom),
                        get_whisk(x, whisker_top),
                        get_whisk(x, whisker_bottom),
                        scoreatpercentile(x, 100))
        bp.draw_on(ax, index, box_color=colour, median_color=colour,
                   whisker_color=colour, blw=box_lw, mlw=median_lw)
