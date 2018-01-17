/*
* chainfksolveracc_recursive.cpp
*
*  Created on: Sep 14, 2012
*      Author: Francisco Vina
*/

/* Copyright (c) 2012, Francisco Vina, CVAP, KTH
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of KTH nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <controllers/chainfksolveracc_recursive.hpp>

namespace KDL {

	ChainFkSolverAcc_recursive::ChainFkSolverAcc_recursive(const Chain &_chain): chain(_chain)
	{

	}

	ChainFkSolverAcc_recursive::~ChainFkSolverAcc_recursive()
	{

	}

	ChainFkSolverAcc::~ChainFkSolverAcc()
	{
	}


	int ChainFkSolverAcc_recursive::JntToCart(const JntArrayAcc &in, FrameAcc &out, int segmentNr)
	{

		if(segmentNr<0)
		segmentNr=chain.getNrOfSegments();

		out=FrameAcc::Identity();

		if(!(in.q.rows()==chain.getNrOfJoints()&&in.qdot.rows()==chain.getNrOfJoints()))
		return -1;
		else if(segmentNr>chain.getNrOfSegments())
		return -1;
		else
		{
			int j=0;
			for (unsigned int i=0;i<segmentNr;i++)
			{
				//Calculate new Frame_base_ee
				if(chain.getSegment(i).getJoint().getType()!=Joint::None){
					out=out*FrameAcc(chain.getSegment(i).pose(in.q(j)),
					chain.getSegment(i).twist(in.q(j),in.qdot(j)),
					SegmentGetTwistAcc(chain.getSegment(i), in.q(j), in.qdot(j), in.qdotdot(j)));
					j++;//Only increase jointnr if the segment has a joint
				}else{
					out=out*FrameAcc(chain.getSegment(i).pose(0.0),
					chain.getSegment(i).twist(0.0,0.0),
					Twist::Zero());
				}
			}
			return 0;
		}
	}
	

	Twist ChainFkSolverAcc_recursive::SegmentGetTwistAcc(const Segment &s, const double &q,const double &qdot, const double &qdotdot) const
	{
		Twist out;
		Joint joint = s.getJoint();
		Frame f_tip = s.getFrameToTip();
		Twist t1 = (joint.twist(qdotdot).RefPoint(joint.pose(q).M * f_tip.p));
		Twist t2 = (joint.twist(qdot).RefPoint(joint.pose(q).M * f_tip.p));
		Twist t3 = (joint.twist(qdot).RefPoint(t2.vel));

		out.rot = t1.rot;
		out.vel = t1.vel+t3.vel;

		return out;
	}


} /* namespace KDL */
