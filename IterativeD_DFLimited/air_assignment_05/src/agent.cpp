/*
 * Daniel Vazquez
 * Aritificial Intelligence for Robotics
 * SS 2016
 * Assignment 5
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
#include <string>
#include <unistd.h>
#include <chrono>
#include <thread>

#define map_rows 25
#define map_cols 141

using namespace std;

Agent::Agent(vector<vector<string> > map, const pair<int, int> initial_pos, int number_of_goals):
empty_map(map),
initial_pos(initial_pos),
number_of_goals(number_of_goals),
max_number_of_stored_nodes(0),
number_of_visited_nodes(0),
total_of_stored_nodes(0),
deepest_level(0),
goal_no(0),
goal_found(false),
no_of_goals_found(0)
{
    max_limit = map_rows * map_cols;
}

Agent::~Agent()
{
}

void Agent::run()
{
    cout << "Running IDFS " << endl;
    cout << "Number of goals " << number_of_goals << endl;
    sleep(1);
    iterative_deepening_search();
}

void Agent::print_map(vector<vector<string> >& a_map)
{
	system("clear");

	for (int row = 0; row < map_rows; row++) {
		for (int col = 0; col < map_cols; col++) {
			cout << a_map[row][col];
		}
		cout << endl;
	}
	this_thread::sleep_for(chrono::milliseconds(10));
}

vector<pair<int, int> > Agent::store_child_nodes(int r, int c, string goal) {
	vector<pair<int, int> > child_nodes;
	// Add all the child nodes, take care of duplicate node
	if (map[r + 1][c].compare(" ") == 0) {
		map[r + 1][c] = "-";
		child_nodes.push_back(make_pair(r + 1, c));
		total_of_stored_nodes++;
	}
	if (map[r - 1][c].compare(" ") == 0) {
		map[r - 1][c] = "-";
		child_nodes.push_back(make_pair(r - 1, c));
		total_of_stored_nodes++;
	}
	if (map[r][c + 1].compare(" ") == 0) {
		map[r][c + 1] = "-";
		child_nodes.push_back(make_pair(r, c + 1));
		total_of_stored_nodes++;
	}
	if (map[r][c - 1].compare(" ") == 0) {
		map[r][c - 1] = "-";
		child_nodes.push_back(make_pair(r, c - 1));
		total_of_stored_nodes++;
	}

	// check for goal nodes, don't change the representation of goal on map
	if (map[r + 1][c].compare(goal) == 0) {
		child_nodes.push_back(make_pair(r + 1, c));
		total_of_stored_nodes++;
	}
	if (map[r - 1][c].compare(goal) == 0) {
		child_nodes.push_back(make_pair(r - 1, c));
		total_of_stored_nodes++;
	}
	if (map[r][c + 1].compare(goal) == 0) {
		child_nodes.push_back(make_pair(r, c + 1));
		total_of_stored_nodes++;
	}
	if (map[r][c - 1].compare(goal) == 0) {
		child_nodes.push_back(make_pair(r, c - 1));
		total_of_stored_nodes++;
	}
	return child_nodes;

}

void Agent::print_vector_contents(vector<pair<int, int> > vec){
	pair<int, int> p;
	for (int i = 0; i < vec.size(); i++){
		p = vec[i];
		cout << p.first <<","<<p.second <<" : ";
	}
}

 bool Agent::recursive_dls(pair<int, int> current_node, int goal, int current_level, int limit, vector<pair<int, int> > current_path)
 {

	int r = current_node.first;
	int c = current_node.second;

	number_of_visited_nodes++;

	string goal_s = to_string(goal);

	if (limit == 0) {
		// Check if its the goal node
		if (map[r][c].compare(goal_s) == 0) {
			initial_pos.first = r;
			initial_pos.second = c;
			map[r][c] = "s";
			goal_path = current_path;
			goal_found = true;
			return true;
		} else {
			return false;
		}
	}

	// check if the current node is the goal
	if (map[r][c].compare(goal_s) == 0) {
		initial_pos.first = r;
		initial_pos.second = c;
		map[r][c] = "s";
		goal_path = current_path;
		goal_found = true;
		return true;
	}
	if (map[r][c].compare("s") == 0) {
		map[r][c] = "-";
	}

	// Add the child nodes
	vector<pair<int, int> > child_nodes = store_child_nodes(r, c, goal_s);

	while (child_nodes.size() > 0) {
		// get the first child
		pair<int, int> child = child_nodes.back();
		child_nodes.pop_back();

		// add the child to the current path
		current_path.push_back(child);

		bool output = recursive_dls(child, goal, current_level + 1, limit - 1,
				current_path);
		if (output)
			break;
	}

	return goal_found;
}

 bool Agent::depth_limited_seach(int limit)
 {
	vector<pair<int, int> > current_path;

	//Initialize variables
	bool found;
	map = empty_map;
	goal_found = false;

	// Declare variables for calling recursive dls
	current_path.push_back(make_pair(initial_pos.first, initial_pos.second));
	pair<int, int> current_node;
	current_node = make_pair(initial_pos.first, initial_pos.second);
	int current_level = 0;

	//Call recursive DLS
	found = recursive_dls(current_node, goal_no, current_level, limit, current_path);

	if (found){
		if (limit > deepest_level){
			deepest_level = limit;
		}
	}
	return found;
}
 void Agent::iterative_deepening_search()
 {
	//TODO

	int n = 1;


	// Iterate over each goal
	for (int n = 1; n <= number_of_goals; n++) {

		bool found = false;
		//Set the goal number
		goal_no = n;

		// increment over depth-limit
		for (int x = 0; x < max_limit; x++) {
			// Call depth-limited-search for each depth
			found = depth_limited_seach(x);
			if (found) {
				break;
			}
		}
		if (found) {
			cout << "Goal " << goal_no << " FOUND " << endl;
			no_of_goals_found++;
			backtrack_path(goal_path);

		} else
			cout << "Goal " << n << " not found" << endl;

	}

	print_final_results();
}

void Agent::print_final_results()
{
    cout << "Deepest level reached: " << deepest_level  << endl;
    cout << "Total of stored nodes: " << total_of_stored_nodes << endl;
    cout << "Total of visited nodes: " << number_of_visited_nodes << endl;
    cout << "Total number of Goals: " << number_of_goals << endl;
    cout << "Number of Goals found: " << no_of_goals_found << endl;
}

void Agent::backtrack_path(vector<pair<int, int> > current_path)
{
	//use the original map to backtrace
    vector<vector<string> > local_map = empty_map;

    while (current_path.size() > 0){

    	pair<int, int> current_data = current_path.back();
    	current_path.pop_back();

    	local_map[current_data.first][current_data.second] = "-";
    }

    //Backtrace. Use the current path vector to set the path on the map.

    print_map(local_map);
    // Adding a 1 sec delay for visualization
    this_thread::sleep_for(chrono::milliseconds(1000));
}
