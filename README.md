# Graph Based Image Segmentation

[![Build Status](https://travis-ci.org/davidstutz/graph-based-image-segmentation.svg?branch=master)](https://travis-ci.org/davidstutz/graph-based-image-segmentation)

**Update:** This implementation is also part of [davidstutz/superpixel-benchmark](https://github.com/davidstutz/superpixel-benchmark).

This repository contains an implementation of the graph-based image segmentation algorithms described in [1] focussing on generating oversegmentations, also referred to as superpixels.

    [1] P. F. Felzenswalb and D. P. Huttenlocher.
        Efficient Graph-Based Image Segmentation.
        International Journal of Computer Vision, volume 59, number 2, 2004.

The implementation was used in [2] for evaluation.

    [2] D. Stutz, A. Hermans, B. Leibe.
        Superpixels: An Evaluation of the State-of-the-Art.
        Computer Vision and Image Understanding, 2018.

![Example: several oversegmentations.](screenshot.png?raw=true "Example: several oversegmentations.")

## Building

The implementation is based on [CMake](https://cmake.org/), [OpenCV](http://opencv.org/) and [Boost](http://www.boost.org/). The following steps have been tested on Ubuntu 12.04:

    $ sudo apt-get install build-essential
    $ sudo apt-get install cmake
    $ sudo apt-get install libboost-all-dev

OpenCV can either be installed following [these instructions](http://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html#linux-installation), or using:

    $ sudo apt-get install libopencv-dev

With all requirements installed, run:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

## Usage

The provided tool can easily be used as follows (from within the `build` directory):

    # Show a help message.
    $ ../bin/refh_cli --help
    Allowed options:
      -h [ --help ]            produce help message
      --input arg              folder containing the images to process
      --threshold arg (=20)    constant for threshold function
      --minimum-size arg (=10) minimum component size
      --output arg (=output)   save segmentation as CSV file and contour images
    # Oversegment the provided examples:
    $ ../bin/refh_cli ../data/ ../output --threshold 255

The latter command will create the `output` directory containing the oversegmentations as `.csv` files and visualizations as `.png` files.

## License

Licenses for source code corresponding to:

D. Stutz, A. Hermans, B. Leibe. **Superpixels: An Evaluation of the State-of-the-Art.** Computer Vision and Image Understanding, 2018.

Note that the two provided images are taken from the [BSDS500](https://www2.eecs.berkeley.edu/Research/Projects/CS/vision/grouping/resources.html).

Copyright (c) 2014-2018 David Stutz, RWTH Aachen University

**Please read carefully the following terms and conditions and any accompanying documentation before you download and/or use this software and associated documentation files (the "Software").**

The authors hereby grant you a non-exclusive, non-transferable, free of charge right to copy, modify, merge, publish, distribute, and sublicense the Software for the sole purpose of performing non-commercial scientific research, non-commercial education, or non-commercial artistic projects.

Any other use, in particular any use for commercial purposes, is prohibited. This includes, without limitation, incorporation in a commercial product, use in a commercial service, or production of other artefacts for commercial purposes.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

You understand and agree that the authors are under no obligation to provide either maintenance services, update services, notices of latent defects, or corrections of defects with regard to the Software. The authors nevertheless reserve the right to update, modify, or discontinue the Software at any time.

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. You agree to cite the corresponding papers (see above) in documents and papers that report on research using the Software.
