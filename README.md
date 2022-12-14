# MVP - Low Level Controller

This repository contains low-level controller for marine vehicles.

## Overview

`mvp_control` is the low level vehicle controller of the MVP framework.
It uses control allocation matrix with quadratic programming optimization.
It highly relies on ROS and ROS transform tree.

![diagram](https://raw.githubusercontent.com/uri-ocean-robotics/mvp_control/master/docs/_static/diagram.svg)

## Documentation

The documentation for `mvp_control` package is is at https://uri-ocean-robotics.github.io/mvp_control.

## Citation

If you find this software useful in your research, please cite:

> Note, this work is published in OCEANS 2022 conference. Once the paper is publicly available, bibtex entry
will be updated with the one from IEEExplore.

```
@inproceedings{mvp_paper,
    title = {Working towards the development of a generic marine vehicle framework: ROS-MVP},
    author={Gezer, Emir Cem and Zhou, Mingxi and Zhao, Lin and McConnell, William},
    journal = {OCEANS 2022, Hampton Road},
    year ={2022},
}
```

## Funding
This work is supported by the [National Science Foundation](https://www.nsf.gov/) award [#2154901](https://www.nsf.gov/awardsearch/showAward?AWD_ID=2154901&HistoricalAwards=false)
