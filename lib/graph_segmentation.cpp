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

#include "graph_segmentation.h"
#include <limits>

void GraphSegmentation::buildGraph(const cv::Mat &image) {
    
    H = image.rows;
    W = image.cols;
    
    int N = H*W;
    graph = ImageGraph(N);
    
    for (int i = 0; i < H; i++) {
        for (int j = 0; j < W; j++) {

            int n = W*i + j;
            ImageNode & node = graph.getNode(n);
            
            cv::Vec3b bgr = image.at<cv::Vec3b>(i, j);
            node.b = bgr[0];
            node.g = bgr[1];
            node.r = bgr[2];

            // Initialize label.
            node.l = n;
            node.id = n;
            node.n = 1;
        }
    }
    
    for (int i = 0; i < H; i++) {
        for (int j = 0; j < W; j++) {
            int n = W*i + j;
            const ImageNode & node = graph.getNode(n);

            if (i < H - 1) {
                int m = W*(i + 1) + j;
                ImageNode & other = graph.getNode(m);

                ImageEdge edge;
                edge.n = n;
                edge.m = m;
                edge.w = (*distance)(node, other);

                graph.addEdge(edge);
            }

            if (j < W - 1) {
                int m = W*i + (j + 1);
                ImageNode & other = graph.getNode(m);

                ImageEdge edge;
                edge.n = n;
                edge.m = m;
                edge.w = (*distance)(node, other);

                graph.addEdge(edge);
            }
        }
    }
}

void GraphSegmentation::oversegmentGraph() {
    
    // Sort edges.
    graph.sortEdges();
    
    for (int e = 0; e < graph.getNumEdges(); e++) {
        ImageEdge edge = graph.getEdge(e%graph.getNumEdges());
        
        ImageNode & n = graph.getNode(edge.n);
        ImageNode & m = graph.getNode(edge.m);

        ImageNode & S_n = graph.findNodeComponent(n);
        ImageNode & S_m = graph.findNodeComponent(m);

        // Are the nodes in different components?
        if (S_m.id != S_n.id) {

            // Here comes the magic!
            if ((*magic)(S_n, S_m, edge)) {
                graph.merge(S_n, S_m, edge);
            }
        }
    }
}

void GraphSegmentation::enforceMinimumSegmentSize(int M) {
    assert(graph.getNumNodes() > 0);
    // assert(graph.getNumEdges() > 0);
    
    for (int e = 0; e < graph.getNumEdges(); e++) {
        ImageEdge edge = graph.getEdge(e);
        
        ImageNode & n = graph.getNode(edge.n);
        ImageNode & m = graph.getNode(edge.m);

        ImageNode & S_n = graph.findNodeComponent(n);
        ImageNode & S_m = graph.findNodeComponent(m);

        if (S_n.l != S_m.l) {
            if (S_n.n < M || S_m.n < M) {
                graph.merge(S_n, S_m, edge);
            }
        }
    }
}

cv::Mat GraphSegmentation::deriveLabels() {
    
    cv::Mat labels(H, W, CV_32SC1, cv::Scalar(0));
    for (int i = 0; i < H; i++) {
        for (int j = 0; j < W; j++) {
            int n = W*i + j;

            ImageNode & node = graph.getNode(n);
            ImageNode & S_node = graph.findNodeComponent(node);

            const int max = std::numeric_limits<int>::max();

            labels.at<int>(i, j) = S_node.id;
        }
    }
    
    return labels;
}