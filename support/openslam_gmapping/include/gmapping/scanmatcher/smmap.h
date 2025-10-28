#ifndef SMMAP_H
#define SMMAP_H
#include <gmapping/grid/map.h>
#include <gmapping/grid/harray2d.h>
#include <gmapping/utils/point.h>
#include <unordered_map>
#include <cstring>
#define SIGHT_INC 1

namespace GMapping {

struct PointAccumulator{
	typedef point<float> FloatPoint;
	/* before 
	PointAccumulator(int i=-1): acc(0,0), n(0), visits(0){assert(i==-1);}
	*/
	/*after begin*/
	PointAccumulator(): acc(0,0), n(0), visits(0){}
	PointAccumulator(int i): acc(0,0), n(0), visits(0){assert(i==-1);}
	/*after end*/
        inline void update(bool value, const Point& p=Point(0,0), int detected=0);
        inline int getLabel();
	inline Point mean() const {return 1./n*Point(acc.x, acc.y);}
	inline operator double() const { return visits?(double)n*SIGHT_INC/(double)visits:-1; }
	inline void add(const PointAccumulator& p) {acc=acc+p.acc; n+=p.n; visits+=p.visits; }
	static const PointAccumulator& Unknown();
	static PointAccumulator* unknown_ptr;
	FloatPoint acc;
	int n, visits;
	inline double entropy() const;

	// Label entropy
	// This is used for semantic mapping, where we want to know the uncertainty
	// of the label assigned to a point.
	// The label is the detected value, which can be a class ID or a semantic label.
	int label_prob[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
	int label_absolute = 0;
	int label_modifier = 6;
	int frees_modifier = 1;
};

void PointAccumulator::update(bool value, const Point& p, int detected){
	if (value) {
		// Assign Detected Value for Semantic Mapping
		int shift_amount = (detected == 100) ? frees_modifier : label_modifier;
		std::memmove(label_prob, label_prob + shift_amount, (12 - shift_amount) * sizeof(int));
		std::fill(label_prob + 12 - shift_amount, label_prob + 12, detected);

		acc.x+= static_cast<float>(p.x);
		acc.y+= static_cast<float>(p.y); 
		n++; 
		visits+=SIGHT_INC;
	} else
		visits++;
}



int PointAccumulator::getLabel() {
	std::unordered_map<int, int> freq;
	for (int i = 0; i < 12; ++i) {
			freq[label_prob[i]]++;
	}

	int mode = label_prob[0];
	int modeCount = freq[mode];

	for (const auto& pair : freq) {
		if (pair.second > modeCount) {
			mode = pair.first;
			modeCount = pair.second;
		}
	}
	// if (mode != 0 && mode != 100 && mode != 101 && mode != 102 && mode != 103) {
	// 	std::cout << "==========ANOMALY GET LABEL=========:::";
	// 	std::cout << mode << std::endl;
	// }
	// return label_absolute;
	return mode;
}

double PointAccumulator::entropy() const{
	if (!visits)
		return -log(.5);
	if (n==visits || n==0)
		return 0;
	double x=(double)n*SIGHT_INC/(double)visits;
	return -( x*log(x)+ (1-x)*log(1-x) );
}


typedef Map<PointAccumulator,HierarchicalArray2D<PointAccumulator> > ScanMatcherMap;

};

#endif 
