// Minimal example: ImGui + ImPlot playback of telemetry_log.txt
// Assumes you have ImGui + ImPlot + SDL2 + OpenGL3 available and linked.

#include <imgui.h>
#include <implot.h>
#include <SDL.h>
#include <SDL_opengl.h>
#include <gl/GL.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <deque>
#include <map>
#include <chrono>
#include <thread>
#include <algorithm>

static const int MAX_POINTS = 2000;

struct Block {
    double timestamp = 0.0;
    std::map<std::string, double> imu;
    std::map<std::string, double> gdc;
    std::map<std::string, double> gps;
};

// parse a single "group: k=v, k=v" line
bool parse_group_line(const std::string &line, std::string &group, std::map<std::string,double> &out) {
    auto pos = line.find(":");
    if (pos == std::string::npos) return false;
    group = line.substr(0, pos);
    std::string rest = line.substr(pos+1);
    // trim
    auto trim = [](std::string &s){
        while (!s.empty() && isspace((unsigned char)s.front())) s.erase(s.begin());
        while (!s.empty() && isspace((unsigned char)s.back())) s.pop_back();
    };
    trim(group);
    trim(rest);
    std::stringstream ss(rest);
    std::string token;
    while (std::getline(ss, token, ',')) {
        trim(token);
        auto eq = token.find('=');
        if (eq == std::string::npos) continue;
        std::string key = token.substr(0, eq);
        std::string val = token.substr(eq+1);
        trim(key); trim(val);
        try {
            double d = std::stod(val);
            out[key] = d;
        } catch(...) {
            // ignore non-numeric
        }
    }
    return true;
}

// Read the telemetry file into blocks (group every N groups into a single Block)
std::vector<Block> read_blocks_from_file(const std::string &path, int groups_per_block=5) {
    std::vector<Block> blocks;
    std::ifstream ifs(path);
    if (!ifs) {
        std::cerr << "Failed to open " << path << "\n";
        return blocks;
    }
    std::string line;
    Block cur;
    int count = 0;
    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        // line begins with ISO timestamp, then comma, then group:
        auto comma = line.find(',');
        if (comma == std::string::npos) continue;
        std::string ts = line.substr(0, comma);
        std::string rest = line.substr(comma+1);
        // parse timestamp approximately into seconds since epoch
        // we'll not be strict; use steady playback spacing instead
        double now = (double)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        std::string group;
        std::map<std::string,double> kv;
        if (parse_group_line(rest, group, kv)) {
            // store in current block
            if (group == "imu") {
                for (auto &kvp: kv) cur.imu[kvp.first] = kvp.second;
            } else if (group == "gdc") {
                for (auto &kvp: kv) cur.gdc[kvp.first] = kvp.second;
            } else if (group == "gps") {
                for (auto &kvp: kv) cur.gps[kvp.first] = kvp.second;
            } else {
                // other groups are stored generically in gps/map if keys match
            }
            cur.timestamp = now;
            count++;
        }
        if (count >= groups_per_block) {
            blocks.push_back(cur);
            cur = Block();
            count = 0;
        }
    }
    // push leftover
    if (count > 0) blocks.push_back(cur);
    return blocks;
}

struct History {
    std::deque<double> t;
    std::deque<double> ax, ay, az;
    std::deque<double> gx, gy, gz;
    std::deque<double> roll, pitch, yaw;
    std::deque<double> pb, ny, nz, throttle;
    std::deque<double> lat, lon, alt;
    void push(double _t, const Block &b) {
        auto pushdq = [&](std::deque<double> &dq, double v){ dq.push_back(v); if ((int)dq.size() > MAX_POINTS) dq.pop_front(); };
        pushdq(t, _t);
        pushdq(ax, b.imu.count("ax_fps2")?b.imu.at("ax_fps2"):NAN);
        pushdq(ay, b.imu.count("ay_fps2")?b.imu.at("ay_fps2"):NAN);
        pushdq(az, b.imu.count("az_fps2")?b.imu.at("az_fps2"):NAN);
        pushdq(gx, b.imu.count("gx_dps")?b.imu.at("gx_dps"):NAN);
        pushdq(gy, b.imu.count("gy_dps")?b.imu.at("gy_dps"):NAN);
        pushdq(gz, b.imu.count("gz_dps")?b.imu.at("gz_dps"):NAN);
        pushdq(roll, b.imu.count("roll_d")?b.imu.at("roll_d"):NAN);
        pushdq(pitch, b.imu.count("pitch_d")?b.imu.at("pitch_d"):NAN);
        pushdq(yaw, b.imu.count("yaw_d")?b.imu.at("yaw_d"):NAN);
        pushdq(pb, b.gdc.count("pbCmd_dps")?b.gdc.at("pbCmd_dps"):NAN);
        pushdq(ny, b.gdc.count("nyCmd_B_fps2")?b.gdc.at("nyCmd_B_fps2"):NAN);
        pushdq(nz, b.gdc.count("nzCmd_B_fps2")?b.gdc.at("nzCmd_B_fps2"):NAN);
        pushdq(throttle, b.gdc.count("throttleCmd_pct")?b.gdc.at("throttleCmd_pct"):NAN);
        pushdq(lat, b.gps.count("latG_d")?b.gps.at("latG_d"):NAN);
        pushdq(lon, b.gps.count("lon_d")?b.gps.at("lon_d"):NAN);
        pushdq(alt, b.gps.count("HAE_ft")?b.gps.at("HAE_ft"):NAN);
    }
};

// helper to convert deque to raw arrays for ImPlot
static std::vector<double> vec_from_deque(const std::deque<double> &dq) {
    return std::vector<double>(dq.begin(), dq.end());
}

int main(int argc, char** argv) {
    // Telemetry file path (relative)
    std::string path = "test/tm/telemetry_log.txt";
    if (argc > 1) path = argv[1];

    // Read blocks
    auto blocks = read_blocks_from_file(path, 5);
    if (blocks.empty()) std::cout << "No blocks read from file: " << path << "\n";

    // Setup SDL + OpenGL + ImGui
    if (SDL_Init(SDL_INIT_VIDEO|SDL_INIT_TIMER|SDL_INIT_GAMECONTROLLER) != 0) {
        std::cerr << "Error: SDL_Init failed\n";
        return -1;
    }
    // GL 3.0 + GLSL 130
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);

    SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    SDL_Window* window = SDL_CreateWindow("Telemetry ImPlot Example", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1200, 900, window_flags);
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO &io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer bindings
    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init("#version 130");

    History hist;
    size_t idx = 0;
    double playback_dt = 0.05; // 20 Hz playback
    auto last_time = std::chrono::steady_clock::now();

    bool show_demo = false;

    while (true) {
        // Poll events
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) goto cleanup;
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE) goto cleanup;
        }

        // Playback one block per playback_dt
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_time).count();
        if (elapsed >= playback_dt) {
            last_time = now;
            if (idx < blocks.size()) {
                double t = (double)std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
                hist.push(t, blocks[idx]);
                idx++;
            }
        }

        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame(window);
        ImGui::NewFrame();

        ImGui::SetNextWindowSize(ImVec2(1200, 900), ImGuiCond_Once);
        ImGui::Begin("Telemetry");

        // Two-column layout - left: euler, gyro, accel; right: lat, lon, alt, gdc
        ImGui::Columns(2, "cols", true);
        // LEFT COLUMN
        ImGui::BeginChild("left", ImVec2(0,0), false);
        if (ImPlot::BeginPlot("Euler Angles", ImVec2(-1,150))) {
            auto vt = vec_from_deque(hist.t);
            auto vroll = vec_from_deque(hist.roll);
            auto vpitch = vec_from_deque(hist.pitch);
            auto vyaw = vec_from_deque(hist.yaw);
            if (!vt.empty()) {
                ImPlot::SetupAxisLimits(ImAxis_X1, vt.front(), vt.back(), ImPlotCond_Always);
            }
            if (!vt.empty() && !vroll.empty()) ImPlot::PlotLine("roll", vt.data(), vroll.data(), (int)vroll.size());
            if (!vt.empty() && !vpitch.empty()) ImPlot::PlotLine("pitch", vt.data(), vpitch.data(), (int)vpitch.size());
            if (!vt.empty() && !vyaw.empty()) ImPlot::PlotLine("yaw", vt.data(), vyaw.data(), (int)vyaw.size());
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot("Gyros", ImVec2(-1,150))) {
            auto vt = vec_from_deque(hist.t);
            auto vgx = vec_from_deque(hist.gx);
            auto vgy = vec_from_deque(hist.gy);
            auto vgz = vec_from_deque(hist.gz);
            if (!vt.empty()) ImPlot::SetupAxisLimits(ImAxis_X1, vt.front(), vt.back(), ImPlotCond_Always);
            if (!vt.empty() && !vgx.empty()) ImPlot::PlotLine("gx", vt.data(), vgx.data(), (int)vgx.size());
            if (!vt.empty() && !vgy.empty()) ImPlot::PlotLine("gy", vt.data(), vgy.data(), (int)vgy.size());
            if (!vt.empty() && !vgz.empty()) ImPlot::PlotLine("gz", vt.data(), vgz.data(), (int)vgz.size());
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot("Accelerations", ImVec2(-1,150))) {
            auto vt = vec_from_deque(hist.t);
            auto vax = vec_from_deque(hist.ax);
            auto vay = vec_from_deque(hist.ay);
            auto vaz = vec_from_deque(hist.az);
            if (!vt.empty()) ImPlot::SetupAxisLimits(ImAxis_X1, vt.front(), vt.back(), ImPlotCond_Always);
            if (!vt.empty() && !vax.empty()) ImPlot::PlotLine("ax", vt.data(), vax.data(), (int)vax.size());
            if (!vt.empty() && !vay.empty()) ImPlot::PlotLine("ay", vt.data(), vay.data(), (int)vay.size());
            if (!vt.empty() && !vaz.empty()) ImPlot::PlotLine("az", vt.data(), vaz.data(), (int)vaz.size());
            ImPlot::EndPlot();
        }
        ImGui::EndChild();

        ImGui::NextColumn();

        // RIGHT COLUMN
        ImGui::BeginChild("right", ImVec2(0,0), false);
        if (ImPlot::BeginPlot("Latitude", ImVec2(-1,120))) {
            auto vt = vec_from_deque(hist.t);
            auto vlat = vec_from_deque(hist.lat);
            if (!vt.empty()) ImPlot::SetupAxisLimits(ImAxis_X1, vt.front(), vt.back(), ImPlotCond_Always);
            if (!vt.empty() && !vlat.empty()) ImPlot::PlotLine("lat", vt.data(), vlat.data(), (int)vlat.size());
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot("Longitude", ImVec2(-1,120))) {
            auto vt = vec_from_deque(hist.t);
            auto vlon = vec_from_deque(hist.lon);
            if (!vt.empty()) ImPlot::SetupAxisLimits(ImAxis_X1, vt.front(), vt.back(), ImPlotCond_Always);
            if (!vt.empty() && !vlon.empty()) ImPlot::PlotLine("lon", vt.data(), vlon.data(), (int)vlon.size());
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot("Altitude", ImVec2(-1,120))) {
            auto vt = vec_from_deque(hist.t);
            auto valt = vec_from_deque(hist.alt);
            if (!vt.empty()) ImPlot::SetupAxisLimits(ImAxis_X1, vt.front(), vt.back(), ImPlotCond_Always);
            if (!vt.empty() && !valt.empty()) ImPlot::PlotLine("alt", vt.data(), valt.data(), (int)valt.size());
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot("GDC Commands", ImVec2(-1,150))) {
            auto vt = vec_from_deque(hist.t);
            auto vpb = vec_from_deque(hist.pb);
            auto vny = vec_from_deque(hist.ny);
            auto vnz = vec_from_deque(hist.nz);
            auto vth = vec_from_deque(hist.throttle);
            if (!vt.empty()) ImPlot::SetupAxisLimits(ImAxis_X1, vt.front(), vt.back(), ImPlotCond_Always);
            if (!vt.empty() && !vpb.empty()) ImPlot::PlotLine("pb", vt.data(), vpb.data(), (int)vpb.size());
            if (!vt.empty() && !vny.empty()) ImPlot::PlotLine("ny", vt.data(), vny.data(), (int)vny.size());
            if (!vt.empty() && !vnz.empty()) ImPlot::PlotLine("nz", vt.data(), vnz.data(), (int)vnz.size());
            if (!vt.empty() && !vth.empty()) ImPlot::PlotLine("throttle", vt.data(), vth.data(), (int)vth.size());
            ImPlot::EndPlot();
        }
        ImGui::EndChild();

        ImGui::Columns(1);

        ImGui::End();

        // Rendering
        ImGui::Render();
        glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);

        // small sleep to avoid pegging CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

cleanup:
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
