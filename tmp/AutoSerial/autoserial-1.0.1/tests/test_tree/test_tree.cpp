#include <math.h>
#include <autoserial/autoserial.h>
#include <iostream>

using namespace std;

static const unsigned nb_branches = 2;

class mean_t : public autoserial::ISerializable {
public:
	AS_CLASSDEF(mean_t)
		AS_MEMBERS
		AS_ARRAY(float, mean, 2)
		AS_CLASSEND;

	mean_t();
};

class node_t;
class cluster_t : public autoserial::ISerializable {
AS_CLASSDEF(cluster_t)
	AS_MEMBERS
	AS_ITEM(mean_t, mean)
	AS_POINTER(node_t, child)
	AS_CLASSEND;
};


class node_t : public autoserial::ISerializable {
AS_CLASSDEF(node_t)
	AS_MEMBERS

	AS_ARRAY(cluster_t, clusters, nb_branches)
	AS_CLASSEND;

	node_t();

};


mean_t::mean_t() {
	//memset(mean,0,2*sizeof(float));
}

node_t::node_t() {
	for (unsigned i=0; i<nb_branches; i++)
		clusters[i].child = 0;
}


int main() {
	const char *filename = "data.bin";


	// construct some data
	node_t * tree = new node_t;

	tree->clusters[0].child = new node_t;
	tree->clusters[1].child = new node_t;
	tree->clusters[1].child->clusters[0].child = new node_t;
	tree->clusters[1].child->clusters[1].child = new node_t;

	// save data
	autoserial::BinaryFileWriter bfw(filename);
	if (AS_FAILED(bfw.write(tree))) {
		cerr << filename << ": unable to write data.\n";
		return -1;
	}

	// check..
	node_t *tree2;
	autoserial::BinaryFileReader bfr(filename);
	if (AS_FAILED(bfr.read((autoserial::ISerializable **)&tree2))) {
		cerr << filename << ": unable to read data.\n";
		return -2;
	}
	if (!tree->equals(tree2)) {
		cerr << "tree comparison failed.\n";
	}
	return 0;
}
