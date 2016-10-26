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

#ifndef GRAPH_SEGMENTATION_H
#define	GRAPH_SEGMENTATION_H

#include <opencv2/opencv.hpp>
#include "image_graph.h"

#define RAND() ((float) std::rand() / (RAND_MAX))

/** \brief Interface to be implemented by a concerete distance. The distance defines
 * how the weights between nodes in the image graph are computed. See the paper
 * by Felzenswalb and Huttenlocher for details. Essentially, derived classes 
 * only need to overwrite the () operator.
 * \author David Stutz
 */
class GraphSegmentationDistance {
public:
    /** \brief Constructor.
     */
    GraphSegmentationDistance() {};
    
    /** \brief Destructor.
     */
    virtual ~GraphSegmentationDistance() {};
    
    /** \brief Compute the distance given 2 nodes.
     * \param[in] n first node
     * \param[in] m second node
     */ 
    virtual float operator()(const ImageNode & n, const ImageNode & m) = 0;
    
};

/** \brief Manhatten (i.e. L1) distance.
 * \author David Stutz
 */
class GraphSegmentationManhattenRGB : public GraphSegmentationDistance {
public:
    /** \brief Constructor; sets normalization constant.
     */
    GraphSegmentationManhattenRGB() {
        // Normalization.
        D = 255 + 255 + 255;
    }
    
    /** \brief Compute the distance given 2 nodes.
     * \param[in] n first node
     * \param[in] m second node
     */ 
    virtual float operator()(const ImageNode & n, const ImageNode & m) {
        float dr = std::abs(n.r - m.r);
        float dg = std::abs(n.g - m.g);
        float db = std::abs(n.b - m.b);
        
        return (dr + dg + db);
    }
    
private:
    
    /** \brief Normalization term. */
    float D;
    
};

/** \brief Euclidean RGB distance.
 * \author David Stutz
 */
class GraphSegmentationEuclideanRGB : public GraphSegmentationDistance {
public:
    /** \brief Constructor; sets normalization constant.
     */
    GraphSegmentationEuclideanRGB() {
        // Normalization.
        D = std::sqrt(255*255 + 255*255 + 255*255);
    }
    
    /** \brief Compute the distance given 2 nodes.
     * \param[in] n first node
     * \param[in] m second node
     */ 
    virtual float operator()(const ImageNode & n, const ImageNode & m) {
        float dr = n.r - m.r;
        float dg = n.g - m.g;
        float db = n.b - m.b;
        
        return std::sqrt(dr*dr + dg*dg + db*db);
    }
    
private:
    
    /** \brief Normalization term. */
    float D;
    
};

/** \brief The magic part of the graph segmentation, i.e. s given two nodes decide
 * whether to add an edge between them (i.e. merge the corresponding segments).
 * See the paper by Felzenswalb and Huttenlocher for details.
 * \author David Stutz
 */
class GraphSegmentationMagic {
public:
    /** \brief Constructor.
     */
    GraphSegmentationMagic() {};
    
    /** \brief Decide whether to merge the two segments corresponding to the
     * given nodes or not.
     * \param[in] S_n node representing the first segment
     * \param[in] S_m node representing the second segment
     * \param[in] e the edge between the two segments
     * \rturn true if merge
     */
    virtual bool operator()(const ImageNode & S_n, const ImageNode & S_m, 
            const ImageEdge & e) = 0;
    
};

/**
 * The original criterion employed by [2].
 */
class GraphSegmentationMagicThreshold : public GraphSegmentationMagic {
public:
    /** \brief Constructor; sets the threshold.
     * \param[in] c the threshold to use
     */
    GraphSegmentationMagicThreshold(float c) : c(c) {};
    
    /** \brief Decide whether to merge the two segments corresponding to the
     * given nodes or not.
     * \param[in] S_n node representing the first segment
     * \param[in] S_m node representing the second segment
     * \param[in] e the edge between the two segments
     * \rturn true if merge
     */
    virtual bool operator()(const ImageNode & S_n, const ImageNode & S_m, 
            const ImageEdge & e) {
        
        float threshold = std::min(S_n.max_w + c/S_n.n, S_m.max_w + c/S_m.n);
        
        if (e.w < threshold) {
            return true;
        }
        
        return false;
    }
    
private:
    
    /** \brief T hreshold. */
    float c;
    
};

/** \brief Implementation of graph based image segmentation as described in the
 * paper by Felzenswalb and Huttenlocher.
 * \author David Stutz
 */
class GraphSegmentation {
public:
    /** \brief Default constructor; uses the Manhatten distance.
     */
    GraphSegmentation() : distance(new GraphSegmentationManhattenRGB()), 
            magic(new GraphSegmentationMagicThreshold(1)) {
        
    };
    
    /** \brief Destructor.
     */
    virtual ~GraphSegmentation() {};
    
    /** \brief Set the distance to use.
     * \param[in] _distance pointer to a GraphSegmentationDistance to use
     */
    void setDistance(GraphSegmentationDistance* _distance) {
        distance = _distance;
    }
    
    /** \brief Set the magic part of graph segmentation.
     * \param[in] _magix pointer to a GraphSegmentationMagic to use
     */
    void setMagic(GraphSegmentationMagic* _magic) {
        magic = _magic;
    }
    
    /** \brief Build the graph nased on the image, i.e. compute the weights
     * between pixels using the underlying distance.
     * \param[in] image image to oversegment
     */
    void buildGraph(const cv::Mat &image);
    
    /** \brief Oversegment the given graph.
     */
    void oversegmentGraph();
    
    /** \brief Enforces the given minimum segment size.
     * \pram[in] M minimum segment size in pixels
     */
    void enforceMinimumSegmentSize(int M);
    
    /** \brief Derive labels from the produced oversegmentation.
     * \return labels as integer matrix
     */
    cv::Mat deriveLabels();
    
protected:
    
    /** \brief Image height. */
    int H;
    
    /** \brief Image widt.h */
    int W;
    
    /** \brief The constructed and segmented image graph. */
    ImageGraph graph;
    
    /** \brief The underlying distance to use. */
    GraphSegmentationDistance* distance;
    
    /** \brief The magic part of graph segmentation. */
    GraphSegmentationMagic* magic;

};

#endif	/* GRAPH_SEGMENTATION_H */

