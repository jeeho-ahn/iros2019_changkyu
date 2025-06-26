// instance_parser.hpp
#pragma once
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#define BOOST_LOG_DYN_LINK 1
#include <boost/log/trivial.hpp>
#define LOG BOOST_LOG_TRIVIAL(trace)

#include "../include/planner.hpp"

#include <config.h>
#include <string>
#include <vector>
#include <tuple>
#include <utility>
#include <sstream>
#include <stdexcept>

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ob = ompl::base;
namespace og = ompl::geometric;

// –– Simple containers for raw data ––//
struct RawObject
{
    std::string name;
    double x, y, theta;
    int num_sides;
};

struct InstanceData
{
    std::vector<RawObject> movables;                              // “mo:…”
    std::vector<std::tuple<double, double, double>> robots;       // “robot:…”
    std::vector<RawObject> goals;                                 // “goal:…”
    std::vector<std::pair<std::string, std::string>> assignments; // “assign:…”
};

// –– Utility split on single char delimiter ––//
inline std::vector<std::string> split(const std::string &s, char delim)
{
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim))
    {
        if (!item.empty())
            out.push_back(item);
    }
    return out;
}

/// Parses **one** instance‐line of the form
///   mo:…;…!robot:…;…!goal:…;…!assign:…;…
/// and returns raw lists of objects, robots, goals, and object→goal pairs.
inline InstanceData parseInstanceLine(const std::string &line)
{
    // 1) split into sections: ["mo:…", "robot:…", "goal:…", "assign:…"]
    auto sections = split(line, '!');
    InstanceData inst;

    for (auto &sec : sections)
    {
        // each sec looks like “mo:entries” or “robot:entries”
        auto kv = split(sec, ':');
        if (kv.size() != 2)
            throw std::runtime_error("Malformed section: " + sec);
        auto &key = kv[0];
        auto &val = kv[1];

        // split entries by ‘;’
        auto entries = split(val, ';');

        if (key == "mo")
        {
            // name,x,y,th,sides
            for (auto &e : entries)
            {
                auto f = split(e, ',');
                if (f.size() != 5)
                    throw std::runtime_error("Bad mo entry: " + e);
                inst.movables.push_back(RawObject{
                    f[0],
                    std::stod(f[1]),
                    std::stod(f[2]),
                    std::stod(f[3]),
                    std::stoi(f[4])});
            }
        }
        else if (key == "robot")
        {
            // x,y,th
            for (auto &e : entries)
            {
                auto f = split(e, ',');
                if (f.size() != 3)
                    throw std::runtime_error("Bad robot entry: " + e);
                inst.robots.emplace_back(
                    std::stod(f[0]),
                    std::stod(f[1]),
                    std::stod(f[2]));
            }
        }
        else if (key == "goal")
        {
            // same format as mo
            for (auto &e : entries)
            {
                auto f = split(e, ',');
                if (f.size() != 5)
                    throw std::runtime_error("Bad goal entry: " + e);
                inst.goals.push_back(RawObject{
                    f[0],
                    std::stod(f[1]),
                    std::stod(f[2]),
                    std::stod(f[3]),
                    std::stoi(f[4])});
            }
        }
        else if (key == "assign")
        {
            // objName,goalName
            for (auto &e : entries)
            {
                auto f = split(e, ',');
                if (f.size() != 2)
                    throw std::runtime_error("Bad assign entry: " + e);
                inst.assignments.emplace_back(f[0], f[1]);
            }
        }
        else
        {
            throw std::runtime_error("Unknown section key: " + key);
        }
    }

    return inst;
}

// read all nonempty lines
inline std::vector<std::string> readFileLines(const std::string &fp)
{
    std::ifstream ifs(fp);
    if (!ifs)
        throw std::runtime_error("Cannot open “" + fp + "”");
    std::vector<std::string> lines;
    for (std::string L; std::getline(ifs, L);)
        if (!L.empty())
            lines.push_back(L);
    return lines;
}

/// Just like allocateAndLoad, but from one “!‐delimited” file
static std::pair<ompl::base::State *, ompl::base::State *>
allocateAndLoadSingle(RobotObjectSetup *env,
                      const std::string &file_path,
                      size_t data_ind,
                      int n_objs)
{
    auto lines = readFileLines(file_path);
    if (data_ind >= lines.size())
        throw std::out_of_range("Index out of range");

    // parse that one line
    InstanceData inst = parseInstanceLine(lines[data_ind]);

    if ((int)inst.movables.size() != n_objs ||
        (int)inst.assignments.size() != n_objs)
    {
        throw std::runtime_error("Expected " +
                                 std::to_string(n_objs) + " movables/assignments, got " +
                                 std::to_string(inst.movables.size()) + " / " +
                                 std::to_string(inst.assignments.size()));
    }

    // build a name→goal map for fast lookup
    std::unordered_map<std::string, RawObject> goal_map;
    for (auto &g : inst.goals)
        goal_map[g.name] = g;

    // allocate
    auto *s_init = env->allocState();
    auto *s_goal = env->allocState();

    // match your old “STATE_ROBOT(s) = …”
    STATE_ROBOT(s_init) = 3;
    STATE_ROBOT(s_goal) = 1;

    // for each object index 1…n_objs
    for (int i = 0; i < n_objs; ++i)
    {
        auto &mo = inst.movables[i];
        // init
        auto *so_i = STATE_OBJECT(s_init, i + 1);
        so_i->setX(mo.x);
        so_i->setY(mo.y);
        so_i->setYaw(mo.theta);
        std::cout << "[INIT] Obj " << (i + 1)
                  << " x=" << mo.x
                  << " y=" << mo.y
                  << " yaw=" << mo.theta
                  << std::endl;

        // find its assigned goal
        auto &ass = inst.assignments[i];
        if (ass.first != mo.name)
            throw std::runtime_error("Assignment order mismatch for " + mo.name);
        auto git = goal_map.find(ass.second);
        if (git == goal_map.end())
            throw std::runtime_error("Missing goal “" + ass.second + "”");

        auto &g = git->second;
        auto *so_g = STATE_OBJECT(s_goal, i + 1);
        so_g->setX(g.x);
        so_g->setY(g.y);
        so_g->setYaw(g.theta);
        std::cout << "[GOAL] Obj " << (i + 1)
                  << " x=" << g.x
                  << " y=" << g.y
                  << " yaw=" << g.theta
                  << std::endl;
    }

    return {s_init, s_goal};
}
