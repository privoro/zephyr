/*
 * Copyright (c) 2018 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <ztest.h>
#include <sys/rb.h>
#include "time.h"

#include "../../../lib/os/rb.c"

#define _CHECK(n) \
	zassert_true(!!(n), "Tree check failed: [ " #n " ] @%d", __LINE__)

#define MAX_NODES 256

static struct rbtree tree;

static struct rbnode nodes[MAX_NODES];

struct container_node {
	struct rbnode node;
	int value;
};

/* Bit is set if node is in the tree */
static unsigned int node_mask[(MAX_NODES + 31)/32];

/* Array of nodes dumped via rb_walk */
static struct rbnode *walked_nodes[MAX_NODES];

/* Node currently being inserted, for testing lessthan() argument order */
static struct rbnode *current_insertee;

void set_node_mask(int node, int val)
{
	unsigned int *p = &node_mask[node / 32];
	unsigned int bit = 1u << (node % 32);

	*p &= ~bit;
	*p |= val ? bit : 0;
}

int get_node_mask(int node)
{
	unsigned int *p = &node_mask[node / 32];
	unsigned int bit = 1u << (node % 32);

	return !!(*p & bit);
}

int node_index(struct rbnode *n)
{
	return (int)(n - &nodes[0]);
}

/* Our "lessthan" is just the location of the struct */
bool node_lessthan(struct rbnode *a, struct rbnode *b)
{
	if (current_insertee) {
		_CHECK(a == current_insertee);
		_CHECK(b != current_insertee);
	}

	return a < b;
}

/* Simple LCRNG (modulus is 2^64!) cribbed from:
 * https://nuclear.llnl.gov/CNP/rng/rngman/node4.html
 *
 * Don't need much in the way of quality, do need repeatability across
 * platforms.
 */
static unsigned int next_rand_mod(unsigned int mod)
{
	static unsigned long long state = 123456789; /* seed */

	state = state * 2862933555777941757ul + 3037000493ul;

	return ((unsigned int)(state >> 32)) % mod;
}

void visit_node(struct rbnode *node, void *cookie)
{
	int *nwalked = cookie;

	_CHECK(*nwalked < MAX_NODES);

	walked_nodes[*nwalked] = node;
	*nwalked += 1;
}

/* Stores the last-seen black height at a leaf during check_rb(), or
 * zero if no leaves have been seen yet
 */
static int last_black_height;

void check_rbnode(struct rbnode *node, int blacks_above)
{
	int side, bheight = blacks_above + z_rb_is_black(node);

	for (side = 0; side < 2; side++) {
		struct rbnode *ch = z_rb_child(node, side);

		if (ch) {
			/* Basic tree requirement */
			if (side == 0) {
				_CHECK(node_lessthan(ch, node));
			} else {
				_CHECK(node_lessthan(node, ch));
			}

			/* Can't have adjacent red nodes */
			_CHECK(z_rb_is_black(node) || z_rb_is_black(ch));

			/* Recurse */
			check_rbnode(ch, bheight);
		} else {
			/* All leaf nodes must be at the same black height */
			if (last_black_height) {
				_CHECK(last_black_height == bheight);
			}
			last_black_height = bheight;
		}
	}
}

void check_rb(void)
{
	last_black_height = 0;

	_CHECK(tree.root);
	_CHECK(z_rb_is_black(tree.root));

	check_rbnode(tree.root, 0);
}

/* First validates the external API behavior via a walk, then checks
 * interior tree and red/black state via internal APIs.
 */
void _check_tree(int size, int use_foreach)
{
	int nwalked = 0, i, ni;
	struct rbnode *n, *last = NULL;

	(void)memset(walked_nodes, 0, sizeof(walked_nodes));

	if (use_foreach) {
		RB_FOR_EACH(&tree, n) {
			visit_node(n, &nwalked);
		}
	} else {
		rb_walk(&tree, visit_node, &nwalked);
	}

	/* Make sure all found nodes are in-order and marked in the tree */
	for (i = 0; i < nwalked; i++) {
		n = walked_nodes[i];
		ni = node_index(n);

		if (last) {
			_CHECK(node_lessthan(last, n));
		}

		_CHECK(get_node_mask(ni));

		last = n;
	}

	/* Make sure all tree bits properly reflect the set of nodes we found */
	ni = 0;
	for (i = 0; i < MAX_NODES; i++) {
		_CHECK(get_node_mask(i) == rb_contains(&tree, &nodes[i]));

		if (get_node_mask(i)) {
			_CHECK(node_index(walked_nodes[ni]) == i);
			ni++;
		}
	}

	_CHECK(ni == nwalked);

	if (tree.root) {
		check_rb();
	}
}

void check_tree(int size)
{
	/* Do it with both enumeration mechanisms */
	_check_tree(size, 0);
	_check_tree(size, 1);
}

void checked_insert(struct rbtree *tree, struct rbnode *node)
{
	current_insertee = node;
	rb_insert(tree, node);
	current_insertee = NULL;
}

void test_tree(int size)
{
	int i, j;

	/* Small trees get checked after every op, big trees less often */
	int small_tree = size <= 32;

	(void)memset(&tree, 0, sizeof(tree));
	tree.lessthan_fn = node_lessthan;
	(void)memset(nodes, 0, sizeof(nodes));
	(void)memset(node_mask, 0, sizeof(node_mask));

	for (j = 0; j < 10; j++) {
		for (i = 0; i < size; i++) {
			int node = next_rand_mod(size);

			if (!get_node_mask(node)) {
				rb_insert(&tree, &nodes[node]);
				set_node_mask(node, 1);
			} else {
				rb_remove(&tree, &nodes[node]);
				set_node_mask(node, 0);
			}

			if (small_tree) {
				check_tree(size);
			}
		}

		if (!small_tree) {
			check_tree(size);
		}
	}
}

void test_rbtree_spam(void)
{
	int size = 1;

	do {
		size += next_rand_mod(size) + 1;

		if (size > MAX_NODES) {
			size = MAX_NODES;
		}

		TC_PRINT("Checking trees built from %d nodes...\n", size);

		test_tree(size);
	} while (size < MAX_NODES);
}

/**
 * @brief Test whether rbtree node struct is embedded
 * in any user struct
 *
 * @details
 * Initialize a user defined struct contains rbtree node
 * Add into a rbtree
 * Enumerate the rbtree node
 *
 * @ingroup lib_rbtree_tests
 *
 * @see RB_FOR_EACH(),RB_FOR_EACH_CONTAINER()
 */
void test_rbtree_container(void)
{
	int count = 0;
	static struct rbtree test_tree_l;
	struct container_node tree_node[10];
	struct container_node *c_foreach_node;
	struct rbnode *foreach_node;

	memset(&test_tree_l, 0, sizeof(test_tree_l));
	test_tree_l.lessthan_fn = node_lessthan;
	memset(tree_node, 0, sizeof(tree_node));
	for (unsigned int i = 0; i < 10; i++) {
		tree_node[i].value = i;
		rb_insert(&test_tree_l, &tree_node[i].node);
	}

	/*"for each" style iteration to verify node*/
	RB_FOR_EACH(&test_tree_l, foreach_node) {
		zassert_true(((struct container_node *)foreach_node)->value
				== count, "RB_FOR_EACH failed");
		count++;
	}

	count = 0;
	/*"for each" style iteration to verify node*/
	RB_FOR_EACH_CONTAINER(&test_tree_l, c_foreach_node, node) {
		zassert_true(c_foreach_node->value == count,
				"RB_FOR_EACH_CONTAINER failed");
		count++;
	}
}



static enum rbtree_perf_stats {
	GET_MIN_MAX, INSERT_REMOVE, NO_OPERATION} operation;
static clock_t time_spent[2][3];
static int nodes_ii[2];
clock_t check_rbtree_perf(struct rbtree *test_tree)
{
	struct rbnode node_test;
	clock_t start = 0, finish = 0;

	start = clock();
	for (int i = 0; i < 10000; i++) {
		switch (operation) {
		case GET_MIN_MAX:
			rb_get_min(test_tree);
			rb_get_max(test_tree);
			break;
		case INSERT_REMOVE:
			rb_insert(test_tree, &node_test);
			rb_remove(test_tree, &node_test);
			break;
		default:
			/*record the time of no operations running*/
			break;
		}
	}
	finish = clock();

	return finish - start;
}

int repeat_operation(int nodes_, int index_)
{
	struct rbnode *node;
	int nodes_real = 0;

	test_tree(nodes_);
	RB_FOR_EACH(&tree, node) {
		nodes_real++;
	}
	operation = GET_MIN_MAX;
	time_spent[index_][0] = check_rbtree_perf(&tree);
	operation = INSERT_REMOVE;
	time_spent[index_][1] = check_rbtree_perf(&tree);
	operation = NO_OPERATION;
	time_spent[index_][2] = check_rbtree_perf(&tree);

	time_spent[index_][0] -= time_spent[index_][2];
	time_spent[index_][1] -= time_spent[index_][2];

	return nodes_real;
}


/**
 * @brief Test rbtree some operations in logarithmic time
 * complex
 *
 * @ingroup lib_rbtree_tests
 *
 * @see rb_get_min(),rb_get_max(),rb_insert(),rb_remove()
 */
void test_check_rbtree_perf(void)
{
	double log_N = 0, get_max_min = 0, insert_remove = 0;

	nodes_ii[0] = repeat_operation(32, 0);
	nodes_ii[1] = repeat_operation(256, 1);

	/*log nodes_ii[1]/nodes_ii[0] = 2.736*/
	log_N = 2.736;

	get_max_min = (double)time_spent[1][0]/time_spent[0][0];
	insert_remove = (double)time_spent[1][1]/time_spent[0][1];

	zassert_within(get_max_min, log_N/2, 0.5, NULL);
	zassert_within(insert_remove, log_N/2, 0.5, NULL);
}

void test_main(void)
{
	ztest_test_suite(test_rbtree,
			 ztest_unit_test(test_rbtree_spam),
			 ztest_1cpu_unit_test(test_rbtree_container),
			 ztest_unit_test(test_check_rbtree_perf)
			 );
	ztest_run_test_suite(test_rbtree);
}
