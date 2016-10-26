/**
 * Copyright (c) 2016, David Stutz
 * Contact: david.stutz@rwth-aachen.de, davidstutz.de
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <fstream>
#include <assert.h>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include "graph_segmentation.h"

/** \brief Read all image files (.png and .jpg) in the given directory.
 * \param[in] directory directory to read
 * \param[out] files found files
 */
void readDirectory(boost::filesystem::path directory,
        std::multimap<std::string, boost::filesystem::path> &files) {
    
    assert(boost::filesystem::is_directory(directory));
    
    files.clear();
    boost::filesystem::directory_iterator end;
    
    for (boost::filesystem::directory_iterator it(directory); it != end; ++it) {
        std::string extension = it->path().extension().string();
        if (extension == ".png" || extension == ".jpg" 
                || extension == ".PNG" || extension == ".JPG") {
            files.insert(std::multimap<std::string, boost::filesystem::path>::value_type(it->path().string(), it->path()));
        }
    }
}

/** \brief Write the given matrix as CSV file.
 * \param[in] file path to file to write
 * \param[in] mat matrix to write, expected to be integer matrix
 */
void writeMatCSV(boost::filesystem::path file, const cv::Mat& mat) {
    
    assert(!mat.empty());
    assert(mat.channels() == 1);
    
    std::ofstream file_stream(file.c_str());
    for (int i = 0; i < mat.rows; i++) {
        for (int j = 0; j < mat.cols; j++) {
            file_stream << mat.at<int>(i, j);
            
            if (j < mat.cols - 1) {
                file_stream << ",";
            }
        }
        
        if (i < mat.rows  - 1) {
            file_stream << "\n";
        }
    }
    
    file_stream.close();
}

/** \brief Check if the given pixel is a boundary pixel in the given 
 * segmentation.
 * \param[in] labels segments as integer image
 * \param[in] i y coordinate
 * \param[in] j x coordinate
 * \return true if boundary pixel, false otherwise
 */
bool is4ConnectedBoundaryPixel(const cv::Mat &labels, int i, int j) {
    
    if (i > 0) {
        if (labels.at<int>(i, j) != labels.at<int>(i - 1, j)) {
            return true;
        }
    }
    
    if (i < labels.rows - 1) {
        if (labels.at<int>(i, j) != labels.at<int>(i + 1, j)) {
            return true;
        }
    }
    
    if (j > 0) {
        if (labels.at<int>(i, j) != labels.at<int>(i, j - 1)) {
            return true;
        }
    }
    
    if (j < labels.cols - 1) {
        if (labels.at<int>(i, j) != labels.at<int>(i, j + 1)) {
            return true;
        }
    }
    
    return false;
}

/** \brief Draw the segments as contours in the image.
 * \param[in] image image to draw contours in (color image expected)
 * \param[in] labels segments to draw as integer image
 * \param[out] contours image with segments indicated by contours
 */
void drawContours(const cv::Mat &image, const cv::Mat &labels, cv::Mat &contours) {
    
    assert(!image.empty());
    assert(image.channels() == 3);
    assert(image.rows == labels.rows && image.cols == labels.cols);
    assert(labels.type() == CV_32SC1);
    
    contours.create(image.rows, image.cols, CV_8UC3);
    cv::Vec3b color(0, 0, 0); // Black contours
    
    for (int i = 0; i < contours.rows; ++i) {
        for (int j = 0; j < contours.cols; ++j) {
            if (is4ConnectedBoundaryPixel(labels, i, j)) {
                
                contours.at<cv::Vec3b>(i, j) = color;
            }
            else {
                contours.at<cv::Vec3b>(i, j) = image.at<cv::Vec3b>(i, j);
            }
        }
    }
}

/** \brief Example of running graph based image segmentation for oversegmentation on
 * a directory possibly containing multiple images. Segmentations are written as CSV
 * and visualizations to the provided output directory.
 *
 * Usage:
 * 
 * \author David Stutz
 */
int main (int argc, char ** argv) {
    
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("input", boost::program_options::value<std::string>(), "folder containing the images to process")
        ("threshold", boost::program_options::value<float>()->default_value(20.0f), "constant for threshold function")
        ("minimum-size", boost::program_options::value<int>()->default_value(10), "minimum component size")
        ("output", boost::program_options::value<std::string>()->default_value("output"), "save segmentation as CSV file and contour images");
    
    boost::program_options::positional_options_description positionals;
    positionals.add("input", 1);
    positionals.add("output", 1);
    
    boost::program_options::variables_map parameters;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(positionals).run(), parameters);
    boost::program_options::notify(parameters);

    if (parameters.find("help") != parameters.end()) {
        std::cout << desc << std::endl;
        return 1;
    }
    
    boost::filesystem::path output_dir(parameters["output"].as<std::string>());
    if (!output_dir.empty()) {
        if (!boost::filesystem::is_directory(output_dir)) {
            boost::filesystem::create_directories(output_dir);
        }
    }
    
    boost::filesystem::path input_dir(parameters["input"].as<std::string>());
    if (!boost::filesystem::is_directory(input_dir)) {
        std::cout << "Image directory not found ..." << std::endl;
        return 1;
    }
    
    float threshold = parameters["threshold"].as<float>();
    int minimum_segment_size = parameters["minimum-size"].as<int>();
    
    std::multimap<std::string, boost::filesystem::path> images;
    readDirectory(input_dir, images);
    
    for (std::multimap<std::string, boost::filesystem::path>::iterator it = images.begin(); 
            it != images.end(); ++it) {
        
        cv::Mat image = cv::imread(it->first);
        
        GraphSegmentationMagicThreshold magic(threshold);
        GraphSegmentationEuclideanRGB distance;
        
        GraphSegmentation segmenter;
        segmenter.setMagic(&magic);
        segmenter.setDistance(&distance);
        
        segmenter.buildGraph(image);
        segmenter.oversegmentGraph();
        segmenter.enforceMinimumSegmentSize(minimum_segment_size);
        
        cv::Mat labels = segmenter.deriveLabels();
        
        boost::filesystem::path csv_file(output_dir 
                / boost::filesystem::path(it->second.stem().string() + ".csv"));
        writeMatCSV(csv_file, labels);
        
        boost::filesystem::path contours_file(output_dir 
                    / boost::filesystem::path(it->second.stem().string() + ".png"));
        cv::Mat image_contours;
        drawContours(image, labels, image_contours);
        cv::imwrite(contours_file.string(), image_contours);
    }
    
    return 0;
}
