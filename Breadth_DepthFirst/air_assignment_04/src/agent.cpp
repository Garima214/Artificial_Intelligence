/*
 * Daniel Vázquez
 * Aritifiacial Intelligence for Robots
 * MAS SS 2016
 * Assignment 3
 *
 * agent.cpp
 * */

/*
 * Team work:
 * Sogol Haghighat and
 * Garima Prasad
 *
 */
#include "agent.hpp"

#include <iostream>
#include <stack>
#include <queue>

#include <chrono>
#include <thread>
#include <time.h>

using namespace std;

Agent::Agent(vector<vector<string> > map, const pair<int, int> initial_pos,
		int number_of_goals, int search_option) :
		map(map), initial_pos(initial_pos), number_of_goals(number_of_goals), search_option(
				search_option), number_of_stored_nodes(0), number_of_visited_nodes(
				0), execution_time(0.0) {

}

Agent::~Agent() {

}

void Agent::run() {

	if (search_option == 1) {
		cout << "Running BFS " << endl;
		cout << "Number of goals " << number_of_goals << endl;
		bfs();
	} else {
		cout << "Running DFS " << endl;
		cout << "Number of goals " << number_of_goals << endl;
		dfs();
	}

}

void Agent::print_map() {
	system("clear");

	for (int row = 0; row < map_rows; row++) {
		for (int col = 0; col < map_cols; col++) {
			cout << map[row][col];
		}
		cout << endl;
	}

	this_thread::sleep_for(chrono::milliseconds(10));

}

void Agent::bfs() {
	//queue stores a pair in the form (row, col)
	queue<pair<int, int> > nodes_queue;
	pair<int, int> current_node;

	clock_t start_time = clock();
	nodes_queue.push(make_pair(initial_pos.first, initial_pos.second));
	number_of_stored_nodes++;
	int count = number_of_goals;

	while (nodes_queue.size() > 0 && count > 0) {

		current_node = nodes_queue.front();
		nodes_queue.pop();
		number_of_visited_nodes++;

		int row = current_node.first;
		int col = current_node.second;

		// checking if the current node is the goal
		if (map[row][col].compare("*") == 0) {
			goal_positions.push_back(make_pair(row, col));
			count = count - 1;
			map[row][col] = "-";
			nodes_queue.push(make_pair(row, col));
		}

		if (map[row][col].compare(" ") == 0) {
			map[row][col] = "-";
			nodes_queue.push(make_pair(row, col));
		}

		//checking the node to the right
		if (map[row + 1][col].compare("*") == 0) {
			goal_positions.push_back(make_pair(row + 1, col));
			count = count - 1;
			map[row + 1][col] = "-";
			nodes_queue.push(make_pair(row + 1, col));
			number_of_stored_nodes++;
		}

		if (map[row + 1][col].compare(" ") == 0) {
			map[row + 1][col] = "-";
			nodes_queue.push(make_pair(row + 1, col));
			number_of_stored_nodes++;
		}

		//checking the node to the left
		if (map[row - 1][col].compare("*") == 0) {
			goal_positions.push_back(make_pair(row - 1, col));
			count = count - 1;
			map[row - 1][col] = "-";
			nodes_queue.push(make_pair(row - 1, col));
			number_of_stored_nodes++;
		}
		if (map[row - 1][col].compare(" ") == 0) {
			map[row - 1][col] = "-";
			nodes_queue.push(make_pair(row - 1, col));
			number_of_stored_nodes++;
		}

		//checking the node at the top
		if (map[row][col - 1].compare("*") == 0) {
			goal_positions.push_back(make_pair(row, col - 1));
			count = count - 1;
			map[row][col - 1] = "-";
			nodes_queue.push(make_pair(row, col - 1));
			number_of_stored_nodes++;
		}
		if (map[row][col - 1].compare(" ") == 0) {
			map[row][col - 1] = "-";
			nodes_queue.push(make_pair(row, col - 1));
			number_of_stored_nodes++;
		}

		//checking the node at the bottom
		if (map[row][col + 1].compare("*") == 0) {
			goal_positions.push_back(make_pair(row, col + 1));
			count = count - 1;
			map[row][col + 1] = "-";
			nodes_queue.push(make_pair(row, col + 1));
			number_of_stored_nodes++;
		}
		if (map[row][col + 1].compare(" ") == 0) {
			map[row][col + 1] = "-";
			nodes_queue.push(make_pair(row, col + 1));
			number_of_stored_nodes++;
		}

		print_map();
	}
	execution_time = (double) (clock() - start_time) * 1000 / CLOCKS_PER_SEC;
	print_evaluation_metrics("queue");
}

void Agent::dfs() {
	//the stack stores a pair in the form (row, col)
	stack<pair<int, int> > nodes_stack;
	pair<int, int> current_node;

	int r, c;
	int goal_count = number_of_goals;

	clock_t start_time = clock();
	nodes_stack.push(make_pair(initial_pos.first, initial_pos.second));
	number_of_stored_nodes++;

	while (nodes_stack.size() > 0) {
		current_node = nodes_stack.top();
		nodes_stack.pop();
		r = current_node.first;
		c = current_node.second;

		// Increment the visited nodes
		number_of_visited_nodes++;

		// Check for dirt
		if (map[r][c].compare("*") == 0 || map[r][c].compare("@") == 0) {
			// Found dirt
			goal_count--;
			goal_positions.push_back(make_pair(r, c));
		}

		// Mark the node as 'visited'
		map[r][c] = '.';

		/*Check the child nodes
		 *Add the child nodes to the stack, only if it is not an obstacle
		 */
		// Check the upper node
		if (map[r - 1][c].compare(" ") == 0) {
			map[r - 1][c] = '#';
			nodes_stack.push(make_pair(r - 1, c));
			number_of_stored_nodes++;
		}
		if (map[r - 1][c].compare("*") == 0) {
			map[r - 1][c] = '@';
			nodes_stack.push(make_pair(r - 1, c));
			number_of_stored_nodes++;
		}

		// Check the lower node
		if (map[r + 1][c].compare(" ") == 0) {
			map[r + 1][c] = '#';
			nodes_stack.push(make_pair(r + 1, c));
			number_of_stored_nodes++;
		}
		if (map[r + 1][c].compare("*") == 0) {
			map[r + 1][c] = '@';
			nodes_stack.push(make_pair(r + 1, c));
			number_of_stored_nodes++;
		}
		// Check the node on the left
		if (map[r][c - 1].compare(" ") == 0) {
			map[r][c - 1] = '#';
			nodes_stack.push(make_pair(r, c - 1));
			number_of_stored_nodes++;
		}
		if (map[r][c - 1].compare("*") == 0) {
			map[r][c - 1] = '@';
			nodes_stack.push(make_pair(r, c - 1));
			number_of_stored_nodes++;
		}

		// Check the node on the right
		if (map[r][c + 1].compare(" ") == 0) {
			map[r][c + 1] = '#';
			nodes_stack.push(make_pair(r, c + 1));
			number_of_stored_nodes++;
		}
		if (map[r][c + 1].compare("*") == 0) {
			map[r][c + 1] = '@';
			nodes_stack.push(make_pair(r, c + 1));
			number_of_stored_nodes++;
		}

		print_map();

		// Stop processing, when all the dirt has been collected
		if (goal_count == 0) {
			break;
		}

	}
	execution_time = (double) (clock() - start_time) * 1000 / CLOCKS_PER_SEC;
	print_evaluation_metrics("stack");

}

void Agent::print_evaluation_metrics(string data_structure_name) {
	print_map();
	cout << "All the possible nodes have been explored " << endl;
	cout << "Found " << goal_positions.size() << " of " << number_of_goals
			<< endl;
	cout << "Maximum size of the " << data_structure_name << " : "
			<< number_of_stored_nodes << endl;
	cout << "Number of stored nodes: " << number_of_stored_nodes << endl;
	cout << "Total of visited nodes: " << number_of_visited_nodes << endl;
	cout << "Execution time (in milliseconds): " << execution_time << endl;
	print_goal_positions();
}

void Agent::print_goal_positions() {
	cout << "Goals: " << endl;
	for (int i = 0; i < goal_positions.size(); i++) {
		cout << i + 1 << ": (" << goal_positions[i].first << ", "
				<< goal_positions[i].second << ")" << endl;
	}
}

