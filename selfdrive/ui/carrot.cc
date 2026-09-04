#include "selfdrive/ui/carrot.h"

#include <cassert>
#include <cmath>
#include <limits>
#include <string>

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QJsonArray>

#include "selfdrive/ui/qt/offroad/model_manager.h"

//#define __TEST
//#define __UI_TEST

#ifdef __APPLE__
#include <OpenGL/gl3.h>
#define NANOVG_GL3_IMPLEMENTATION
#define nvgCreate nvgCreateGL3
#else
#include <GLES3/gl3.h>
#define NANOVG_GLES3_IMPLEMENTATION
#define nvgCreate nvgCreateGLES3
#endif

#define NANOVG_GLES3_IMPLEMENTATION
#include <nanovg_gl.h>
#include <nanovg_gl_utils.h>

#define COLOR_BLACK nvgRGBA(0, 0, 0, 255)
#define COLOR_BLACK_ALPHA(x) nvgRGBA(0, 0, 0, x)
#define COLOR_WHITE nvgRGBA(255, 255, 255, 255)
#define COLOR_WHITE_ALPHA(x) nvgRGBA(255, 255, 255, x)
#define COLOR_RED_ALPHA(x) nvgRGBA(255, 0, 0, x)
#define COLOR_YELLOW nvgRGBA(218, 202, 37, 255)
#define COLOR_YELLOW_ALPHA(x) nvgRGBA(218, 202, 37, x)
//#define COLOR_RED nvgRGBA(201, 34, 49, 255)
#define COLOR_RED nvgRGBA(255, 0, 0, 255)
#define COLOR_OCHRE nvgRGBA(218, 111, 37, 255)
#define COLOR_OCHRE_ALPHA(x) nvgRGBA(218, 111, 37, x)
#define COLOR_GREEN nvgRGBA(0, 203, 0, 255)
#define COLOR_GREEN_ALPHA(x) nvgRGBA(0, 153, 0, x)
#define COLOR_BLUE nvgRGBA(0, 0, 255, 255)
#define COLOR_BLUE_ALPHA(x) nvgRGBA(0, 0, 255, x)
#define COLOR_ORANGE nvgRGBA(255, 175, 3, 255)
#define COLOR_ORANGE_ALPHA(x) nvgRGBA(255, 175, 3, x)
#define COLOR_GREY nvgRGBA(191, 191, 191, 1)
#define COLOR_GREY_ALPHA(x) nvgRGBA(191, 191, 191, x)

#define BOLD "KaiGenGothicKR-Bold"//"Inter-Bold"//"sans-bold"


constexpr float MIN_DRAW_DISTANCE = 10.0;
constexpr float MAX_DRAW_DISTANCE = 100.0;

ModelRenderer* _model = NULL;
extern  int get_path_length_idx(const cereal::XYZTData::Reader& line, const float path_height);
int g_fps= 0;

static void ui_draw_text(const UIState* s, float x, float y, const char* string, float size, NVGcolor color, const char* font_name, float borderWidth=3.0, float shadowOffset=0.0, NVGcolor borderColor=COLOR_BLACK, NVGcolor shadowColor=COLOR_BLACK) {
    y += 6;
    nvgFontFace(s->vg, font_name);
    nvgFontSize(s->vg, size);
    if (borderWidth > 0.0) {
        //NVGcolor borderColor = COLOR_BLACK;
        nvgFillColor(s->vg, borderColor);
        for (int i = 0; i < 360; i += 45) {
            float angle = i * NVG_PI / 180.0f;
            float offsetX = borderWidth * cos(angle);
            float offsetY = borderWidth * sin(angle);
            nvgText(s->vg, x + offsetX, y + offsetY, string, NULL);
        }
    }
    if (shadowOffset != 0.0) {
        //NVGcolor shadowColor = COLOR_BLACK;
        nvgFillColor(s->vg, shadowColor);
        nvgText(s->vg, x + shadowOffset, y + shadowOffset, string, NULL);
    }
    nvgFillColor(s->vg, color);
    nvgText(s->vg, x, y, string, NULL);
}
static void ui_draw_text_vg(NVGcontext* vg, float x, float y, const char* string, float size, NVGcolor color, const char* font_name, float borderWidth = 3.0, float shadowOffset = 0.0, NVGcolor borderColor = COLOR_BLACK, NVGcolor shadowColor = COLOR_BLACK) {
    //y += 6;
    nvgFontFace(vg, font_name);
    nvgFontSize(vg, size);
    if (borderWidth > 0.0) {
        //NVGcolor borderColor = COLOR_BLACK;
        nvgFillColor(vg, borderColor);
        for (int i = 0; i < 360; i += 45) {
            float angle = i * NVG_PI / 180.0f;
            float offsetX = borderWidth * cos(angle);
            float offsetY = borderWidth * sin(angle);
            nvgText(vg, x + offsetX, y + offsetY, string, NULL);
        }
    }
    if (shadowOffset != 0.0) {
        //NVGcolor shadowColor = COLOR_BLACK;
        nvgFillColor(vg, shadowColor);
        nvgText(vg, x + shadowOffset, y + shadowOffset, string, NULL);
    }
    nvgFillColor(vg, color);
    nvgText(vg, x, y, string, NULL);
}
void ui_draw_line_vg(NVGcontext* vg, const QPolygonF& vd, NVGcolor* color, NVGpaint* paint, float stroke = 0.0, NVGcolor strokeColor = COLOR_WHITE) {
    if (vd.size() == 0) return;

    nvgBeginPath(vg);
    nvgMoveTo(vg, vd.at(0).x(), vd.at(0).y());
    for (int i = 1; i < vd.size(); i++) {
        nvgLineTo(vg, vd.at(i).x(), vd.at(i).y());
    }
    nvgClosePath(vg);
    if (color) {
        nvgFillColor(vg, *color);
    }
    else if (paint) {
        nvgFillPaint(vg, *paint);
    }
    nvgFill(vg);
    if (stroke > 0.0) {
        nvgStrokeColor(vg, strokeColor);
        nvgStrokeWidth(vg, stroke);
        nvgStroke(vg);
    }
}
void ui_draw_line2_vg(NVGcontext* vg, float x[], float y[], int size, NVGcolor* color, NVGpaint* paint, float stroke = 0.0, NVGcolor strokeColor = COLOR_WHITE) {

    nvgBeginPath(vg);
    nvgMoveTo(vg, x[0], y[0]);
    for (int i = 1; i < size; i++) {
        nvgLineTo(vg, x[i], y[i]);
    }
    nvgClosePath(vg);
    if (color) {
        nvgFillColor(vg, *color);
    }
    else if (paint) {
        nvgFillPaint(vg, *paint);
    }
    nvgFill(vg);

    if (stroke > 0.0) {
        nvgStrokeColor(vg, strokeColor);
        nvgStrokeWidth(vg, stroke);
        nvgStroke(vg);
    }
}

void ui_draw_image(const UIState* s, const Rect1& r, const char* name, float alpha) {
    nvgBeginPath(s->vg);
    NVGpaint imgPaint = nvgImagePattern(s->vg, r.x, r.y, r.w, r.h, 0, s->images.at(name), alpha);
    nvgRect(s->vg, r.x, r.y, r.w, r.h);
    nvgFillPaint(s->vg, imgPaint);
    nvgFill(s->vg);
}
void ui_draw_line(const UIState* s, const QPolygonF& vd, NVGcolor* color, NVGpaint* paint, float stroke = 0.0, NVGcolor strokeColor = COLOR_WHITE) {
    if (vd.size() == 0) return;

    nvgBeginPath(s->vg);
    nvgMoveTo(s->vg, vd.at(0).x(), vd.at(0).y());
    for (int i = 1; i < vd.size(); i++) {
        nvgLineTo(s->vg, vd.at(i).x(), vd.at(i).y());
    }
    nvgClosePath(s->vg);
    if (color) {
        nvgFillColor(s->vg, *color);
    }
    else if (paint) {
        nvgFillPaint(s->vg, *paint);
    }
    nvgFill(s->vg);
    if (stroke > 0.0) {
        nvgStrokeColor(s->vg, strokeColor);
        nvgStrokeWidth(s->vg, stroke);
        nvgStroke(s->vg);
    }
}
void ui_draw_line2(const UIState* s, float x[], float y[], int size, NVGcolor* color, NVGpaint* paint, float stroke = 0.0, NVGcolor strokeColor = COLOR_WHITE) {

    nvgBeginPath(s->vg);
    nvgMoveTo(s->vg, x[0], y[0]);
    for (int i = 1; i < size; i++) {
        nvgLineTo(s->vg, x[i], y[i]);
    }
    nvgClosePath(s->vg);
    if (color) {
        nvgFillColor(s->vg, *color);
    }
    else if (paint) {
        nvgFillPaint(s->vg, *paint);
    }
    nvgFill(s->vg);

    if (stroke > 0.0) {
        nvgStrokeColor(s->vg, strokeColor);
        nvgStrokeWidth(s->vg, stroke);
        nvgStroke(s->vg);
    }
}
#if 0
void ui_draw_rect(NVGcontext* vg, const Rect1& r, NVGcolor color, int width, float radius) {
    nvgBeginPath(vg);
    radius > 0 ? nvgRoundedRect(vg, r.x, r.y, r.w, r.h, radius) : nvgRect(vg, r.x, r.y, r.w, r.h);
    nvgStrokeColor(vg, color);
    nvgStrokeWidth(vg, width);
    nvgStroke(vg);
}
#endif

static inline void fill_rect(NVGcontext* vg, const Rect1& r, const NVGcolor* color, const NVGpaint* paint,
    float radius, float stroke_width, NVGcolor* stroke_color) {
    nvgBeginPath(vg);

    if (radius > 0) {
        nvgRoundedRect(vg, r.x, r.y, r.w, r.h, radius);
    }
    else {
        nvgRect(vg, r.x, r.y, r.w, r.h);
    }

    if (color) nvgFillColor(vg, *color);
    if (paint) nvgFillPaint(vg, *paint);
    nvgFill(vg);

    if (stroke_width > 0) {
        nvgStrokeWidth(vg, stroke_width);
        if (stroke_color) nvgStrokeColor(vg, *stroke_color);
		else nvgStrokeColor(vg, nvgRGB(0, 0, 0));   
        nvgStroke(vg);                         
    }
}

void ui_fill_rect(NVGcontext* vg, const Rect1& r, const NVGcolor& color, float radius, float stroke_width, NVGcolor* stroke_color) {
    fill_rect(vg, r, &color, nullptr, radius, stroke_width, stroke_color);
}

void ui_fill_rect(NVGcontext* vg, const Rect1& r, const NVGpaint& paint, float radius, float stroke_width, NVGcolor* stroke_color) {
    fill_rect(vg, r, nullptr, &paint, radius, stroke_width, stroke_color);
}


////////////////////  draw_text animated
float a_x = 0.;
float a_y = 0.;
float a_size = 0.;
const int a_max = 100;
char a_font[128] = "";
int a_time = -1;
char a_string[256] = "";
NVGcolor a_color = COLOR_WHITE;
static void ui_draw_text_a2(const UIState* s) {
    if (a_time <= 0) return;
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
    a_time -= 10;
    int a_time1 = a_time;
    if (a_time1 > 100) a_time1 = 100;
    int x = (s->fb_w / 2 * a_time1 + a_x * (a_max - a_time1)) / a_max;
    int y = ((s->fb_h - 400) * a_time1 + a_y * (a_max - a_time1)) / a_max;
    int size = (350 * a_time1 + a_size * (a_max - a_time1)) / a_max;
    if (a_time >= 100) ui_draw_text(s, x, y, a_string, size, a_color, a_font, 9.0, 8.0, COLOR_BLACK, COLOR_BLACK);
    else ui_draw_text(s, x, y, a_string, size, a_color, a_font);
}
static void ui_draw_text_a(const UIState* s, float x, float y, const char* string, float size, NVGcolor color, const char* font_name) {
    a_x = x;
    a_y = y;
    strcpy(a_string, string);
    a_size = size;
    a_color = color;
    strcpy(a_font, font_name);
    a_time = 130;
}
////////////////////  draw_text animated


//##################### drawPlot
#define PLOT_MAX 400 //500
class DrawPlot : public QObject {
    Q_OBJECT
protected:
    int     plotSize = 0;
    int     plotIndex = 0;
    float   plotQueue[3][PLOT_MAX];
    float   plotMin = 0.;
    float   plotMax = 0.;
    float   plotShift = 0.0;
    float   plotX = 350.0;// 30.0;// 300.0;
    float   plotWidth = 1000;
    float   plotY = 40;// 70.0;// 120.0;// 30.0;
    float   plotHeight = 300.0;
    float   plotRatio = 1.0;
    float   plotDx = 2.0;
    int     show_plot_mode = 0;
    int     show_plot_mode_prev = -1;
    void	drawPlotting(const UIState* s, int index, int start, float x, float y[], int size, NVGcolor* color, float stroke = 0.0) {
        nvgBeginPath(s->vg);
        plotRatio = (plotMax - plotMin) < 1.0 ? plotHeight : plotHeight / (plotMax - plotMin);
        float dx = plotDx;
        char str[128];

        for (int i = 0; i < size; i++) {
            float data = y[(start - i + PLOT_MAX) % PLOT_MAX];
            float plot_y = plotY + plotHeight - (data - plotMin) * plotRatio;
            if (i == 0) {
                nvgMoveTo(s->vg, x + (size - i) * dx, plot_y);
                sprintf(str, "%.2f", data);
                ui_draw_text(s, x + (size - i) * dx + 50, plot_y + ((index > 0) ? 40 : 0), str, 40, *color, BOLD);
            }
            else nvgLineTo(s->vg, x + (size - i) * dx, plot_y);
        }
        //nvgClosePath(s->vg);

        if (stroke > 0.0) {
            nvgStrokeColor(s->vg, *color);
            nvgStrokeWidth(s->vg, stroke);
            nvgStroke(s->vg);
        }

    }
    Params  params;
    std::deque<float> minDeque[3];  // 최소값을 유지하는 덱
    std::deque<float> maxDeque[3];  // 최대값을 유지하는 덱
    void	makePlotData(const UIState* s, float data[], char* title) {

        SubMaster& sm = *(s->sm);
        auto    car_state = sm["carState"].getCarState();
        auto    lp = sm["longitudinalPlan"].getLongitudinalPlan();
        auto    car_control = sm["carControl"].getCarControl();
        auto    controls_state = sm["controlsState"].getControlsState();
        auto    torque_state = controls_state.getLateralControlState().getTorqueState();

        float   a_ego = car_state.getAEgo();
        float   v_ego = car_state.getVEgo();

        float   accel = lp.getAccels()[0];
        float   speeds_0 = lp.getSpeeds()[0];

        float   accel_out = car_control.getActuators().getAccel();

        const cereal::ModelDataV2::Reader& model = sm["modelV2"].getModelV2();
        const auto position = model.getPosition();
        const auto velocity = model.getVelocity();

        auto lead_radar = sm["radarState"].getRadarState().getLeadOne();
        auto live_params = sm["liveParameters"].getLiveParameters();

        // 0: yellow, 1: green, 2: orange
        switch (show_plot_mode) {
        case 0:
        case 1:
            data[0] = a_ego;
            data[1] = accel;
            data[2] = accel_out;
            sprintf(title, "1.Accel (Y:a_ego, G:a_target, O:a_out)");
            break;
        case 2:
            data[0] = speeds_0;
            data[1] = v_ego;
            data[2] = a_ego;
            sprintf(title, "2.Speed/Accel(Y:speed_0, G:v_ego, O:a_ego)");
            break;
        case 3:
            data[0] = position.getX()[32];
            data[1] = velocity.getX()[32];
            data[2] = velocity.getX()[0];
            sprintf(title, "3.Model(Y:pos_32, G:vel_32, O:vel_0)");
			      break;
        case 4:
            data[0] = accel;
            data[1] = lead_radar.getALeadK();
            data[2] = lead_radar.getVRel();
            sprintf(title, "4.Lead(Y:accel, G:a_lead, O:v_rel)");
            break;
        case 5:
            data[0] = a_ego;
            data[1] = lead_radar.getALead();
            data[2] = lead_radar.getJLead();
            sprintf(title, "5.Lead(Y:a_ego, G:a_lead, O:j_lead)");
            break;
        case 6:
            data[0] = torque_state.getActualLateralAccel() * 10.0;
            data[1] = torque_state.getDesiredLateralAccel() * 10.0;
            data[2] = torque_state.getOutput() * 10.0;
            sprintf(title, "6.Steer(Y:actual, G:desire, O:output)");
            break;
        case 7:
            data[0] = car_state.getSteeringAngleDeg();
            data[1] = car_control.getActuators().getSteeringAngleDeg();
            data[2] = live_params.getAngleOffsetDeg() * 10.0;
            sprintf(title, "7.SteerA (Y:Actual, G:Target, O:Offset*10)");
            break;
        case 8:
            data[0] = car_control.getActuators().getCurvature()*10000;
            data[1] = car_control.getActuators().getCurvature()*10000;
            data[2] = car_control.getActuators().getCurvature()*10000;
            sprintf(title, "8.SteerA (Y:Actual, G:Target, O:Offset*10)");
            break;
        default:
            data[0] = data[1] = data[2] = 0;
            sprintf(title, "no data");
            break;
        }
        if (show_plot_mode != show_plot_mode_prev) {
            plotSize = 0;
            plotIndex = 0;
            plotMin = 0.;
            plotMax = 0.;
            for (int i = 0; i < 3; i++) {
              minDeque[i].clear();
              maxDeque[i].clear();
            }
            show_plot_mode_prev = show_plot_mode;
        }
    }

    void updatePlotQueue(float plot_data[3]) {
        // plotIndex 업데이트
        plotIndex = (plotIndex + 1) % PLOT_MAX;

        for (int i = 0; i < 3; i++) {
            // 오래된 값이 덱의 최솟값/최댓값에 해당하면 제거
            if (plotSize == PLOT_MAX) {
                if (!minDeque[i].empty() && minDeque[i].front() == plotQueue[i][plotIndex]) {
                    minDeque[i].pop_front();
                }
                if (!maxDeque[i].empty() && maxDeque[i].front() == plotQueue[i][plotIndex]) {
                    maxDeque[i].pop_front();
                }
            }

            // 새 값을 큐에 추가
            plotQueue[i][plotIndex] = plot_data[i];

            // minDeque 업데이트
            while (!minDeque[i].empty() && minDeque[i].back() > plot_data[i]) {
                minDeque[i].pop_back();
            }
            minDeque[i].push_back(plot_data[i]);

            // maxDeque 업데이트
            while (!maxDeque[i].empty() && maxDeque[i].back() < plot_data[i]) {
                maxDeque[i].pop_back();
            }
            maxDeque[i].push_back(plot_data[i]);
        }

        // 큐 크기 증가
        if (plotSize < PLOT_MAX) {
            plotSize++;
        }

        // 전체 최소값 및 최대값 계산
        plotMin = std::numeric_limits<double>::max();
        plotMax = std::numeric_limits<double>::lowest();
        for (int i = 0; i < 3; i++) {
            plotMin = std::min(plotMin, minDeque[i].front());
            plotMax = std::max(plotMax, maxDeque[i].front());
        }

        // plotMin 및 plotMax 범위 제한
        if (plotMin > -2.0) plotMin = -2.0;
        if (plotMax < 2.0) plotMax = 2.0;
    }
public:
    void	draw(const UIState* s) {
        show_plot_mode = params.getInt("ShowPlotMode");
        if (show_plot_mode == 0) return;
        SubMaster& sm = *(s->sm);
        if (sm.alive("carState") && sm.alive("longitudinalPlan"));
        else return;

        //ui_fill_rect(s->vg, { (int)plotX - 10, (int)plotY - 50, (int)(PLOT_MAX * plotDx) + 150, (int)plotHeight + 100}, COLOR_BLACK_ALPHA(90), 30);


        float plot_data[3] =  {0., 0., 0. };
        char title[128] = "";

        makePlotData(s, plot_data, title);

        updatePlotQueue(plot_data);

        /*
        for (int i = 0; i < 3; i++) {
            if (plotMin > plot_data[i]) plotMin = plot_data[i];
            if (plotMax < plot_data[i]) plotMax = plot_data[i];
        }
        if (plotMin > -2.) plotMin = -2.0;
        if (plotMax < 2.0) plotMax = 2.0;

        plotIndex = (plotIndex + 1) % PLOT_MAX;
        for(int i=0;i<3;i++) plotQueue[i][plotIndex] = plot_data[i];
        if (plotSize < PLOT_MAX - 1) plotSize++;
        */
        if (s->fb_w < 1200) return;

        NVGcolor color[3] = { COLOR_YELLOW, COLOR_GREEN, COLOR_ORANGE };
        for (int i = 0; i < 3; i++) {
            drawPlotting(s, i, plotIndex, plotX, plotQueue[i], plotSize, &color[i], 3.0f);
        }
        ui_draw_text(s, plotX + 400, plotY - 20, title, 25, COLOR_WHITE, BOLD);
    }
};

class ModelDrawer : public QObject{
      Q_OBJECT
protected:
    template <class T>
    float interp(float x, std::initializer_list<T> x_list, std::initializer_list<T> y_list, bool extrapolate)
    {
        std::vector<T> xData(x_list);
        std::vector<T> yData(y_list);
        int size = xData.size();

        int i = 0;
        if (x >= xData[size - 2]) {
            i = size - 2;
        }
        else {
            while (x > xData[i + 1]) i++;
        }
        T xL = xData[i], yL = yData[i], xR = xData[i + 1], yR = yData[i + 1];
        if (!extrapolate) {
            if (x < xL) yR = yL;
            if (x > xR) yL = yR;
        }

        T dydx = (yR - yL) / (xR - xL);
        return yL + dydx * (x - xL);
    }
    template <class T>
    float interp(float x, const T* x_list, const T* y_list, size_t size, bool extrapolate)
    {
        int i = 0;
        if (x >= x_list[size - 2]) {
            i = size - 2;
        }
        else {
            while (x > x_list[i + 1]) i++;
        }
        T xL = x_list[i], yL = y_list[i], xR = x_list[i + 1], yR = y_list[i + 1];
        if (!extrapolate) {
            if (x < xL) yR = yL;
            if (x > xR) yL = yR;
        }

        T dydx = (yR - yL) / (xR - xL);
        return yL + dydx * (x - xL);
    }
    void update_line_data(const UIState* s, const cereal::XYZTData::Reader& line,
        float y_off, float z_off_left, float z_off_right, QPolygonF* pvd, int max_idx, bool allow_invert = true, float y_shift = 0.0, int start_idx = 0) {
        const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();
        QPolygonF left_points, right_points;
        left_points.reserve(max_idx + 1);
        right_points.reserve(max_idx + 1);

        for (int i = start_idx; i <= max_idx; i++) {
            // highly negative x positions  are drawn above the frame and cause flickering, clip to zy plane of camera
            if (line_x[i] < 0) continue;
            QPointF left, right;
            bool l = _model->mapToScreen(line_x[i], line_y[i] - y_off + y_shift, line_z[i] + z_off_left, &left);
            bool r = _model->mapToScreen(line_x[i], line_y[i] + y_off + y_shift, line_z[i] + z_off_right, &right);
            if (l && r) {
                // For wider lines the drawn polygon will "invert" when going over a hill and cause artifacts
                if (!allow_invert && left_points.size() && left.y() > left_points.back().y()) {
                    continue;
                }
                left_points.push_back(left);
                right_points.push_front(right);
            }
        }
        *pvd = left_points + right_points;
    }
};
class PathEndDrawer : ModelDrawer {
private:
    QPointF path_end_left_vertex;
    QPointF path_end_right_vertex;
    int     radarTrackId = -1;
    float   alpha = 0.85;
    float   path_fx = 0.0;
    float   path_fy = 0.0;
    float   path_fwidth = 0.0;

    int     path_x = 0;
    int     path_y = 0;
    int     path_width = 0;
    bool    lead_status = false;
    float   radarDist = 0.0;
    float   visionDist = 0.0;
    int     xState = 0;
    int     trafficState = 0;

    float   v_ego = 0.0;
    bool    brakeHoldActive = false;
    int    softHoldActive = 0;
    int    carrotCruise = 0;
    bool    longActive = false;

    float   t_follow = 0.0;
    float   tf_distance = 0.0;
    QPointF tf_vertex_left;
    QPointF tf_vertex_right;

    QPointF lead_two_left;
    QPointF lead_two_right;
    float   lead_two_xl = 0.0;
    float   lead_two_xr = 0.0;
    float   lead_two_y = 0.0;
    int     lead_two_status = 0;

protected:
    bool make_data(const UIState* s) {
		SubMaster& sm = *(s->sm);
		if (!sm.alive("modelV2")) return false;

        v_ego = sm["carState"].getCarState().getVEgo();
        brakeHoldActive = sm["carState"].getCarState().getBrakeHoldActive();
        softHoldActive = sm["carState"].getCarState().getSoftHoldActive();
        carrotCruise = sm["carState"].getCarState().getCarrotCruise();
        auto selfdrive_state = sm["selfdriveState"].getSelfdriveState();
        longActive = selfdrive_state.getEnabled();
        //longActive = sm["carControl"].getCarControl().getLongActive();
        //longActive = sm["carControl"].getCarControl().getEnabled();
        auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
        xState = lp.getXState();
        trafficState = lp.getTrafficState();
        const cereal::ModelDataV2::Reader& model = sm["modelV2"].getModelV2();
        auto line = model.getPosition();
        float max_distance = s->max_distance;
        int idx = get_path_length_idx(line, max_distance);
        float y = line.getY()[idx];
        float z = line.getZ()[idx];

        auto leadsV3 = model.getLeadsV3()[0];
        visionDist = leadsV3.getProb() > .5 ? leadsV3.getX()[0] - 1.52 : 0.0;

        auto lead_one = sm["radarState"].getRadarState().getLeadOne();
        lead_status = lead_one.getStatus();
        if (lead_status) {
            z = line.getZ()[get_path_length_idx(line, lead_one.getDRel())];
            max_distance = lead_one.getDRel();
            y = -lead_one.getYRel();
            radarTrackId = lead_one.getRadarTrackId();
            radarDist = (lead_one.getRadar()) ? lead_one.getDRel() : 0;
        }
        else {
            radarTrackId = -1;
            radarDist = 0;
        }
        _model->mapToScreen(max_distance, y - 1.2, z + 1.22, &path_end_left_vertex);
        _model->mapToScreen(max_distance, y + 1.2, z + 1.22, &path_end_right_vertex);

        auto lead_two = sm["radarState"].getRadarState().getLeadTwo();
        if (lead_two.getRadar() && lead_two.getDRel() > lead_one.getDRel() + 3.0) {
          z = line.getZ()[get_path_length_idx(line, lead_two.getDRel())];
          y = -lead_two.getYRel();

          _model->mapToScreen(lead_two.getDRel(), y - 1.2, z + 1.22, &lead_two_left);
          _model->mapToScreen(lead_two.getDRel(), y + 1.2, z + 1.22, &lead_two_right);
          if (lead_two_status > 0) {
            lead_two_xl = lead_two_xl * 0.8 + lead_two_left.x() * 0.2;
            lead_two_xr = lead_two_xr * 0.8 + lead_two_right.x() * 0.2;
            lead_two_y = lead_two_y * 0.8 + lead_two_left.y() * 0.2;
          }
          else {
            lead_two_xl = lead_two_left.x();
            lead_two_xr = lead_two_right.x();
            lead_two_y = lead_two_left.y();
          }
          if(lp.getLongitudinalPlanSource() == cereal::LongitudinalPlan::LongitudinalPlanSource::LEAD1) {
            lead_two_status = 2;
          }
          else {
            lead_two_status = 1;
          }
        }
        else {
          lead_two_status = 0;
        }

        float lex = path_end_left_vertex.x();
        float ley = path_end_left_vertex.y();
        float rex = path_end_right_vertex.x();
        float rey = path_end_right_vertex.y();

        float _path_width = rex - lex;
        float _path_x = (lex + rex) / 2;
        float _path_y = (ley + rey) / 2;
        _path_x = (int)std::clamp(_path_x, 350.f, s->fb_w - 350.f);
        _path_y = (int)std::clamp(_path_y, 200.f, s->fb_h - 80.f);
        if (isnan(_path_x) || isnan(_path_y) || isnan(_path_width));
        else {
            path_fx = path_fx * alpha + _path_x * (1. - alpha);
            path_fy = path_fy * alpha + _path_y * (1. - alpha);

            if (_path_width < 120.) _path_width = 120.;
            else if (_path_width > 800.) _path_width = 800.;
            path_fwidth = path_fwidth * alpha + _path_width * (1. - alpha);
        }
        path_x = (int)path_fx;
        path_y = (int)path_fy;
        path_width = (int)path_fwidth;


        t_follow = lp.getTFollow();
        tf_distance = lp.getDesiredDistance(); // t_follow* v_ego + 6;
        int tf_idx = get_path_length_idx(line, tf_distance);
        float tf_y = line.getY()[tf_idx];
        float tf_z = line.getZ()[tf_idx];
        _model->mapToScreen(tf_distance, tf_y - 1.0, tf_z + 1.22, &tf_vertex_left);
        _model->mapToScreen(tf_distance, tf_y + 1.0, tf_z + 1.22, &tf_vertex_right);

        return true;
	};
    bool isLeadSCC() {
        return radarTrackId < 1;
    }
    bool isRadarDetected() {
        return radarTrackId >= 0;
    }
    bool isLeadDetected() {
        return lead_status;
    }
public:
    int getPathX() { return path_x; }
    int getPathY() { return path_y; }

    void draw(const UIState* s) {
		if (!make_data(s)) return;
        nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
        int x = path_x;
        int y = path_y - 135;
        char str[128];
        int disp_y = y + 195;// 175;
        bool draw_dist = false;
        float disp_size = 50;
        if (softHoldActive || brakeHoldActive || carrotCruise) {
            sprintf(str, "%s", (brakeHoldActive) ? "AUTOHOLD" : (softHoldActive) ? "SOFTHOLD": "CARROT");
            ui_draw_text(s, x, disp_y, str, disp_size, COLOR_WHITE, BOLD);
        }
        else if (longActive) {
            if (xState == 3 || xState == 5) {      //XState.e2eStop, XState.e2eStopped
                if (v_ego < 1.0) {
                    sprintf(str, "%s", (trafficState >= 1000) ? tr("Signal Error").toStdString().c_str(): tr("Signal Ready").toStdString().c_str());
                    ui_draw_text(s, x, disp_y, str, disp_size, COLOR_WHITE, BOLD);
                }
                else {
                    ui_draw_text(s, x, disp_y, tr("Signal slowing").toStdString().c_str(), disp_size, COLOR_WHITE, BOLD);
                }
#if 0
                else if (getStopDist() > 0.5) {
                    float dist = getStopDist() * (s->scene.is_metric ? 1 : METER_TO_FOOT);
                    if (dist < 10.0) sprintf(str, "%.1fM", dist);
                    else sprintf(str, "%.0fM", dist);
                    ui_draw_text(s, x, disp_y, str, disp_size, COLOR_WHITE, BOLD);
                }
#endif
            }
            else if (xState == 4) {     //XState.e2ePrepare
				      ui_draw_text(s, x, disp_y, "E2E주행중", disp_size, COLOR_WHITE, BOLD);
			      }
            else if (xState == 0 || xState == 1 || xState == 2) {     //XState.lead
                draw_dist = true;
            }
        }
        else draw_dist = true;

        if (draw_dist) {
            //float dist = (getRadarDist() > 0.0) ? getRadarDist() : getVisionDist();
            //if (dist < 10.0) sprintf(str, "%.1f", dist);
            //else sprintf(str, "%.0f", dist);
            //ui_draw_text(s, x, disp_y, str, disp_size, COLOR_WHITE, BOLD);
            int wStr = 0, w = 80;
            float dist = radarDist * (s->scene.is_metric ? 1 : METER_TO_FOOT);
            NVGcolor text_color = (xState==0) ? COLOR_WHITE : (xState==1) ? COLOR_GREY : COLOR_GREEN;
            if (dist > 0.0) {
                sprintf(str, "%.1f", dist);
                wStr = 32 * (strlen(str) + 0);
                ui_fill_rect(s->vg, { (int)(x - w - wStr / 2), (int)(disp_y - 35), wStr, 42 }, isLeadSCC() ? COLOR_RED : COLOR_ORANGE, 15);
                ui_draw_text(s, x - w, disp_y, str, 40, text_color, BOLD);
            }
            dist = visionDist * (s->scene.is_metric ? 1 : METER_TO_FOOT);
            if (dist > 0.0) {
                sprintf(str, "%.1f", dist);
                wStr = 32 * (strlen(str) + 0);
                ui_fill_rect(s->vg, { (int)(x + w - wStr / 2), (int)(disp_y - 35), wStr, 42 }, COLOR_BLUE, 15);
                ui_draw_text(s, x + w, disp_y, str, 40, text_color, BOLD);
            }
        }
        QPolygonF tf_vertext;
        if (tf_distance > 0) {
          tf_vertext.push_back(tf_vertex_left);
          tf_vertext.push_back(tf_vertex_right);
          ui_draw_line(s, tf_vertext, nullptr, nullptr, 3.0, COLOR_WHITE);
          sprintf(str, "%.1f(%.2f)", tf_distance, t_follow);
          ui_draw_text(s, tf_vertex_right.x(), tf_vertex_right.y(), str, 25, COLOR_WHITE, BOLD);
        }


        float px[7], py[7];
        NVGcolor rcolor = isLeadSCC() ? COLOR_RED : COLOR_ORANGE;
        NVGcolor  pcolor = !isRadarDetected() ? ((trafficState == 1) ? rcolor : COLOR_GREEN) : isRadarDetected() ? rcolor : COLOR_BLUE;
        bool show_path_end = true;
        if (false && show_path_end && !isLeadDetected()) {
            px[0] = path_x - path_width / 2;
            px[1] = path_x + path_width / 2;
            px[2] = path_x + path_width / 2;
            px[3] = path_x + 20;
            px[4] = path_x;
            px[5] = path_x - 20;
            px[6] = path_x - path_width / 2;
            py[0] = path_y;
            py[1] = path_y;
            py[2] = path_y - 7;
            py[3] = path_y - 17;
            py[4] = path_y - 0;
            py[5] = path_y - 17;
            py[6] = path_y - 7;
            ui_draw_line2(s, px, py, 7, &pcolor, nullptr, 3.0f);
        }
        if (isLeadDetected()) {
            NVGcolor radar_stroke = COLOR_BLUE;            
            if (lead_two_status > 0) {
              radar_stroke = COLOR_OCHRE;
              int path_width2 = lead_two_xr - lead_two_xl;
              ui_fill_rect(
                s->vg,
                { (int)(lead_two_xl - 10), (int)(lead_two_y - path_width2 * 0.8), (int)(path_width2 + 20), (int)(path_width2 * 0.8) },
                (lead_two_status == 2) ? COLOR_RED_ALPHA(50) : COLOR_BLACK_ALPHA(20),
                15, 3, &radar_stroke
              );
            }
            radar_stroke = isRadarDetected() ? rcolor : COLOR_BLUE;
            ui_fill_rect(s->vg, { (int)(path_x - path_width / 2 - 10), (int)(path_y - path_width * 0.8), (int)(path_width + 20), (int)(path_width * 0.8) }, COLOR_BLACK_ALPHA(20), 15, 3, &radar_stroke);
#if 0
            px[0] = path_x - path_width / 2 - 10;
            px[1] = px[0] + path_width + 20;
            px[2] = px[1];
            px[3] = px[0];
            py[0] = path_y + 20;// 5;
            py[1] = py[0];
            py[2] = py[1] - path_width * 0.8;
            py[3] = py[2];
            NVGcolor color2 = COLOR_BLACK_ALPHA(20);
            ui_draw_line2(s, px, py, 4, &color2, nullptr, 3.0f, isRadarDetected() ? rcolor : COLOR_BLUE);
#endif

#if 0
            auto lead_radar = sm["radarState"].getRadarState().getLeadOne();
            float h = path_width * 0.8 / 2.;
            float ax[4], ay[4], vx[4], vy[4];
            vx[0] = px[0];
            vx[1] = vx[0] + 20;
            vx[2] = vx[1];
            vx[3] = vx[0];
            vy[0] = py[0] - h;
            vy[1] = vy[0];
            float v = lead_radar.getVRel() / 5.0 * h;
            if (v < -h) v = -h;
            if (v > h) v = h;
            vy[2] = vy[1] - v;
            vy[3] = vy[2];
            NVGcolor vcolor = isLeadSCC() ? COLOR_RED : COLOR_ORANGE;
            ui_draw_line2(s, vx, vy, 4, &vcolor, nullptr, 0.0f);

            ax[0] = px[1];
            ax[1] = px[1] - 20;
            ax[2] = ax[1];
            ax[3] = ax[0];
            ay[0] = py[0] - h;
            ay[1] = ay[0];
            float a = lead_radar.getALeadK() / 2.0 * h;
            if (a < -h) a = -h;
            if (a > h) a = h;
            ay[2] = ay[1] - a;
            ay[3] = ay[2];
            NVGcolor acolor = isLeadSCC() ? COLOR_RED : COLOR_ORANGE;
            ui_draw_line2(s, ax, ay, 4, &acolor, nullptr, 0.0f);
#endif
        }

	}
};

class LaneLineDrawer : ModelDrawer {
private:
    float lane_line_probs[4];
    float road_edge_stds[2];
    QPolygonF lane_line_vertices[4];
    QPolygonF lane_line_vertices_for_double;
    QPolygonF road_edge_vertices[2];
    int  left_lane_line = 0;
    int  right_lane_line = 0;

protected:
    bool make_data(const UIState* s) {
        SubMaster& sm = *(s->sm);
        if (!sm.alive("modelV2")) return false;
        if (!sm.alive("carState")) return false;
        const cereal::ModelDataV2::Reader& model = sm["modelV2"].getModelV2();
        const auto model_lane_lines = model.getLaneLines();
        const auto model_lane_line_probs = model.getLaneLineProbs();
        int max_idx = get_path_length_idx(model_lane_lines[0], s->max_distance);
        left_lane_line = sm["carState"].getCarState().getLeftLaneLine();
        right_lane_line = sm["carState"].getCarState().getRightLaneLine();
        for (int i = 0; i < std::size(lane_line_vertices); i++) {
            lane_line_probs[i] = model_lane_line_probs[i];
            float line_width = 0.025;
            if (i == 1 && left_lane_line >= 20) line_width = 0.05;
            update_line_data(s, model_lane_lines[i], line_width, 0.0, 0.0, &lane_line_vertices[i], max_idx);
            if (i == 1) {
              update_line_data(s, model_lane_lines[i], line_width, 0.0, 0.0, &lane_line_vertices_for_double, max_idx, true, -0.3);
            }
            //update_line_data(s, model_lane_lines[i], line_width * lane_line_probs[i], 0.0, 0.0, &lane_line_vertices[i], max_idx);
            //if (i == 1) {
            //  update_line_data(s, model_lane_lines[i], line_width * lane_line_probs[i], 0.0, 0.0, &lane_line_vertices_for_double, max_idx, true, -0.3);
            //}
        }

        // roadedges
        const auto model_road_edges = model.getRoadEdges();
        const auto model_road_edge_stds = model.getRoadEdgeStds();
        int max_idx_road_edge = get_path_length_idx(model_lane_lines[0], 100);
        for (int i = 0; i < std::size(road_edge_vertices); i++) {
            road_edge_stds[i] = model_road_edge_stds[i];
            update_line_data(s, model_road_edges[i], 0.025, 0.0, 0.0, &road_edge_vertices[i], max_idx_road_edge);
        }
        return true;
    }
    void drawRoadEdge(const UIState* s) {
        NVGcolor color;
        for (int i = 0; i < std::size(road_edge_vertices); ++i) {
            float temp_f = std::clamp<float>(road_edge_stds[i] / 2.0, 0.0, 1.0);
            color = nvgRGBAf(1.0 - temp_f, 0.0, temp_f, 1.0);
            ui_draw_line(s, road_edge_vertices[i], &color, nullptr);
        }
    }

public:
    void draw(const UIState* s, int show_lane_info) {
        if (show_lane_info < 1) return;
        if(!make_data(s)) return;
        NVGcolor color;
        for (int i = 0; i < std::size(lane_line_vertices); ++i) {
          int alpha = (lane_line_probs[i] > 0.3) ? 220 : 0;
          int stroke = 0.0;
          if (i == 1) {
            color = (left_lane_line >= 20) ? COLOR_YELLOW_ALPHA(alpha) : COLOR_WHITE_ALPHA(alpha);
            stroke = (left_lane_line >= 20) ? 1.0 : 0.0;
          }
          else if (i == 2) color = (right_lane_line >= 20) ? COLOR_YELLOW_ALPHA(alpha) : COLOR_WHITE_ALPHA(alpha);
          else color = COLOR_WHITE_ALPHA(alpha);
          ui_draw_line(s, lane_line_vertices[i], &color, nullptr, stroke);
          if ((i == 1) && (left_lane_line%10 == 4)) {
            ui_draw_line(s, lane_line_vertices_for_double, &color, nullptr, stroke);
          }
        }
        if(show_lane_info > 1) drawRoadEdge(s);
    }
};
class TurnInfoDrawer : ModelDrawer {
private:
    int xSpdLimit = 0;
    int xSpdDist = 0;
    int xSignType = -1;
    int xTurnInfo = -1;
    int xDistToTurn = 0;
    int nRoadLimitSpeed = 20;
    int active_carrot = 0;

    int nGoPosDist = 0;
    int nGoPosTime = 0;
    int nVTurnSpeed = 0; // [243차] TBT HUD route= 아래줄 vturn= 표시용

    QString szSdiDescr = "";
    QString atc_type;
    QString szPosRoadName = "";
    QString szTBTMainText = "";

protected:
    QPointF navi_turn_point[2];
    float navi_turn_point_x[2] = { 0.0, };
    float navi_turn_point_y[2] = { 0.0, };
    bool navi_turn_point_flag = true;
    void drawTurnInfo(const UIState* s) {
        nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_BOTTOM);
        if (xDistToTurn < 1500 && xDistToTurn > 0) {
            SubMaster& sm = *(s->sm);

            //const auto carrot_man = sm["carrotMan"].getCarrotMan();
            //szTBTMainText = QString::fromStdString(carrot_man.getSzTBTMainText());

            const cereal::ModelDataV2::Reader& model = sm["modelV2"].getModelV2();
            const auto road_edges = model.getRoadEdges();
            int idx = get_path_length_idx(road_edges[0], xDistToTurn);
            int m_idx[2] = { 0, 1 };
            for (int i = 0; i < 2; i++) {
                int m = m_idx[i];
                _model->mapToScreen(road_edges[m].getX()[idx], road_edges[m].getY()[idx], road_edges[m].getZ()[idx], &navi_turn_point[i]);
            }

            float scale = 0.2;
            if (xDistToTurn < 200) scale = 1.0 - (0.8 * xDistToTurn / 200.);
            scale *= 0.5;
            int size_x = 348 * scale;
            int size_y = 540 * scale;
            int img_x = 0;
            int img_y = 0;
            float alpha = (navi_turn_point_flag) ? 1.0 : 0.1;

            for (int i = 0; i < 2; i++) {
                navi_turn_point_x[i] = navi_turn_point_x[i] * (1.0 - alpha) + navi_turn_point[i].x() * alpha;
                navi_turn_point_y[i] = navi_turn_point_y[i] * (1.0 - alpha) + navi_turn_point[i].y() * alpha;
            }
            navi_turn_point_flag = false;

            switch (xTurnInfo) {
            case 1: case 3: case 5:
                img_x = (int)navi_turn_point_x[0] - size_x / 2;
                img_y = (int)navi_turn_point_y[0] - size_y;
                ui_draw_image(s, { img_x, img_y, size_x, size_y }, "ic_navi_point", 1.0f);
                nvgTextAlign(s->vg, NVG_ALIGN_RIGHT | NVG_ALIGN_BOTTOM);
                break;
            case 2: case 4:
                img_x = (int)navi_turn_point_x[1] - size_x / 2;
                img_y = (int)navi_turn_point_y[1] - size_y;
                ui_draw_image(s, { img_x, img_y, size_x, size_y }, "ic_navi_point", 1.0f);
                nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_BOTTOM);
                break;
            }
            char str[128] = "";
            sprintf(str, "%s", szTBTMainText.toStdString().c_str());
            float text_scale = (scale>0.5)?scale:0.5;
            ui_draw_text(s, img_x + size_x / 2, img_y + size_y + 100 * text_scale, str, 100 * text_scale, COLOR_WHITE, BOLD);
        }
        else navi_turn_point_flag = true;
    }
    QPointF left_dist_point;
    float left_dist_x = 0.0;
    float left_dist_y = 0.0;
    bool left_dist_flag = true;

    void drawSpeedLimit(const UIState* s) {
        nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
        if (xSpdLimit <= 0) left_dist_flag = true;
        if (xSpdDist > 0) {
            SubMaster& sm = *(s->sm);
            const cereal::ModelDataV2::Reader& model = sm["modelV2"].getModelV2();
            const auto road_edges = model.getRoadEdges();
            const auto lane_lines = model.getLaneLines();
            char str[128] = "";

            if (xSpdDist < 100) {
                int idx = get_path_length_idx(lane_lines[2], xSpdDist);
                _model->mapToScreen(lane_lines[2].getX()[idx], lane_lines[2].getY()[idx], lane_lines[2].getZ()[idx], &left_dist_point);
            }
            else {
                int idx = get_path_length_idx(road_edges[0], xSpdDist);
                _model->mapToScreen(road_edges[1].getX()[idx], road_edges[1].getY()[idx], road_edges[1].getZ()[idx], &left_dist_point);
            }

            float scale = 0.6;
            if (xSpdDist < 200) scale = 1.0 - (0.6 * xSpdDist / 200.);
            int bx = left_dist_point.x() + 140 * scale;
            int by = left_dist_point.y();
            if (left_dist_flag) {
                left_dist_x = bx;
                left_dist_y = by;
            }
            else {
                left_dist_x = left_dist_x * 0.9 + bx * 0.1;
                left_dist_y = left_dist_y * 0.9 + by * 0.1;
            }

            bx = left_dist_x;
            by = left_dist_y;
            left_dist_flag = false;

            if (xSpdDist > 0) {

                if (s->scene.is_metric) {
                    if (xSpdDist < 1000) sprintf(str, "%d m", xSpdDist);
                    else  sprintf(str, "%.1f km", xSpdDist / 1000.f);
                }
                else {
                    // 1 미터 = 3.28084 피트로 계산
                    if (xSpdDist < 1609) sprintf(str, "%d ft", (int)(xSpdDist * 3.28084)); // 1609m(1마일)보다 작으면 피트로 표시
                    else sprintf(str, "%.1f mi", xSpdDist / 1609.34f); // 1609m 이상이면 마일로 표시
                }
                ui_draw_text(s, bx, by + 120 * scale, str, 40 * scale, COLOR_WHITE, BOLD);
            }

            if (xSignType == 22) {
                ui_draw_image(s, { bx - (int)(60 * scale), by - (int)(50 * scale), (int)(120 * scale), (int)(150 * scale) }, "ic_speed_bump", 1.0f);
            }
            else {
                nvgBeginPath(s->vg);
                nvgCircle(s->vg, bx, by, 140 / 2 * scale);
                nvgFillColor(s->vg, COLOR_WHITE);
                nvgFill(s->vg);
                nvgBeginPath(s->vg);
                nvgCircle(s->vg, bx, by, 130 / 2 * scale);
                nvgFillColor(s->vg, COLOR_RED);
                nvgFill(s->vg);
                nvgBeginPath(s->vg);
                nvgCircle(s->vg, bx, by, 110 / 2 * scale);
                nvgFillColor(s->vg, COLOR_WHITE);
                nvgFill(s->vg);
                sprintf(str, "%d", (int)(xSpdLimit * ((s->scene.is_metric)?1:KM_TO_MILE) + 0.5));
                ui_draw_text(s, bx, by + 25 * scale - 6 * (1 - scale), str, 60 * scale, COLOR_BLACK, BOLD, 0.0f, 0.0f);
            }
        }
	}
    // ===== 경로안내창(TBT HUD) 크기 설정 =====
    // 100차 이전: 폭 790px (화면 절반 가까이 차지, 차량 시야 가림 큼)
    // 변경: 폭만 460px로 축소 (높이 300px는 유지 — 목표 크기 실측 결과 폭 차이가 지배적)
    static constexpr int TBT_BOX_W = 460;   // 기존 790
    static constexpr int TBT_BOX_H = 340;   // 154차: 300->340. 하단 도로명/route 텍스트가
                                             // 1줄->2줄로 늘어나며 필요한 세로 여백 40px 추가
    static constexpr int TBT_MARGIN_R = 10; // 우측 여백 (기존과 동일)
    static constexpr int TBT_ICON_SIZE = 140; // 기존 icon_size(256) 대신 축소 전용 크기 사용
    // [230차] 하단 도로명 줄바꿈용: 줄바꿈 1줄당 필요한 세로 높이 / 최대 줄 수.
    // 도로명이 길어 2줄로 줄바꿈되면 박스 높이를 이만큼 위쪽으로 확장한다
    // (박스 하단 좌표는 고정, 상단만 위로 이동 — 다른 요소 위치 영향 없음).
    static constexpr int TBT_LINE_STEP = 40;
    static constexpr int TBT_MAX_NAME_LINES = 2;
    // [231차, 사용자 지시] route 숫자 한 요소만 별도 배율(1.5x->2x)로 강조하는
    // 대신, 경로안내창(TBT HUD) 박스 내부의 모든 텍스트(상단 안내문구,
    // 회전/차선변경 라벨, 잔여거리, ETA, 도로명, route=prefix+숫자)에
    // 동일한 배율을 곱해 전체 글자 크기를 일괄 확대한다. prefix 대비 숫자를
    // 더 크게 보이게 하는 기존 2배 비율(154/156차 기원) 자체는 유지하고,
    // 그 위에 이 배율을 얹는다.
    static constexpr float TBT_FONT_SCALE = 1.3f;

    int  drawTurnInfoHud(const UIState* s) {
      if (s->fb_w < 1200) return -1;
#ifdef __UI_TEST
        active_carrot = 2;
        nGoPosDist = 500000;
        nGoPosTime = 4 * 60 * 60;
        szSdiDescr = "어린이 보호구역(스쿨존 시작 구간)";
        xTurnInfo = 1;
        xDistToTurn = 1000;
        szPosRoadName = "구문천 1길 17";
#endif

        //if (active_carrot <= 1) return;
        //printf("nGoPosDist=%d, nGoPosTime=%d\n", nGoPosDist, nGoPosTime);

        //if (xDistToTurn <= 0 || nGoPosDist <= 0) return;
        char str[128] = "";

        int tbt_x = s->fb_w - (TBT_BOX_W + TBT_MARGIN_R);
        int tbt_y = s->fb_h - 250;
        int right_x = tbt_x + TBT_BOX_W - 20; // [230차] 하단 두 줄(도로명/route=) 우측끝 정렬 기준 x좌표
        NVGcolor stroke_color = COLOR_WHITE;
        // [231차] 이 함수 안의 모든 폰트 크기는 기존(스케일 적용 전) 값에
        // TBT_FONT_SCALE(1.3배)을 곱해서 사용한다 (반올림).
        auto FS = [](int base) { return (int)(base * TBT_FONT_SCALE + 0.5f); };

        // [230차] 도로명(szPosRoadName 1줄)이 박스 폭을 넘으면 최대
        // TBT_MAX_NAME_LINES줄까지 자동 줄바꿈. 아래 박스를 그리기 전에 먼저
        // 계산해서, 줄바꿈이 실제로 필요한 만큼만 박스 높이를 위로 넉넉하게 잡는다.
        auto wrap_name_lines = [&](const std::string& txt, float avail_w) {
          std::vector<std::string> lines;
          int fs = FS(30);
          for (; fs >= FS(20); fs -= 3) {
            nvgFontSize(s->vg, fs);
            nvgFontFace(s->vg, BOLD);
            float bounds[4];
            nvgTextBounds(s->vg, 0, 0, txt.c_str(), NULL, bounds);
            if (bounds[2] - bounds[0] <= avail_w) {
              lines.clear();
              lines.push_back(txt);
              break;
            }
            NVGtextRow rows[TBT_MAX_NAME_LINES];
            int nrows = nvgTextBreakLines(s->vg, txt.c_str(), NULL, avail_w, rows, TBT_MAX_NAME_LINES);
            bool covers_all = (nrows > 0) && (rows[nrows - 1].end == txt.c_str() + txt.size());
            if (covers_all || fs <= FS(20)) {
              lines.clear();
              for (int i = 0; i < nrows; i++) lines.push_back(std::string(rows[i].start, rows[i].end));
              break;
            }
          }
          return std::make_pair(fs, lines);
        };

        std::string name_line1, name_line2;
        std::vector<std::string> name_wrapped;
        int name_fs = FS(30);
        int tbt_extra_h = 0;
        if (szSdiDescr.length() == 0 && szPosRoadName.length() > 0) {
          std::string full = szPosRoadName.toStdString();
          size_t nl = full.find('\n');
          name_line1 = (nl == std::string::npos) ? full : full.substr(0, nl);
          name_line2 = (nl == std::string::npos) ? "" : full.substr(nl + 1);
          if (name_line1.length() > 0) {
            auto result = wrap_name_lines(name_line1, TBT_BOX_W - 40);
            name_fs = result.first;
            name_wrapped = result.second;
            if (name_wrapped.size() > 1) {
              tbt_extra_h = (int)(name_wrapped.size() - 1) * TBT_LINE_STEP;
            }
          }
        }

        if (s->scene._current_carrot_display == 3) {
          ui_fill_rect(s->vg, { tbt_x, 5, TBT_BOX_W, s->fb_h - 15 }, COLOR_BLACK_ALPHA(120), 30, 2, &stroke_color);
        }
        if (nGoPosDist > 0 && nGoPosTime > 0);
        else return -1;
        if (s->scene._current_carrot_display == 3);
        else {
          // [230차] 상단만 tbt_extra_h만큼 위로 확장(하단 좌표는 그대로 유지)
          ui_fill_rect(s->vg, { tbt_x, tbt_y - 60 - tbt_extra_h, TBT_BOX_W, TBT_BOX_H + tbt_extra_h }, COLOR_BLACK_ALPHA(120), 30, 2, &stroke_color);
        }
        // 상단: 안내 문구 (예: "분기점")
        if (szTBTMainText.length() > 0) {
          nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_BOTTOM);
          ui_draw_text(s, tbt_x + 20, tbt_y - 15, szTBTMainText.toStdString().c_str(), FS(32), COLOR_WHITE, BOLD);
        }

        // 좌측: 회전/분기 아이콘 + 잔여거리 (세로 배치, 아이콘 축소)
        int bx = tbt_x + 85;
        int by = tbt_y + 80;
        if(xTurnInfo > 0) {
            nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
            if (atc_type.length() > 0) {
              stroke_color = COLOR_BLACK;
              ui_fill_rect(s->vg, { bx - 65, by - 75, 130, 175 }, atc_type.contains("prepare")?COLOR_GREEN_ALPHA(100) : COLOR_GREEN, 12, 1.0f, &stroke_color);
            }
            switch (xTurnInfo) {
            case 1: ui_draw_image(s, { bx - TBT_ICON_SIZE / 2, by - TBT_ICON_SIZE / 2, TBT_ICON_SIZE, TBT_ICON_SIZE }, "ic_turn_l", 1.0f); break;
            case 2: ui_draw_image(s, { bx - TBT_ICON_SIZE / 2, by - TBT_ICON_SIZE / 2, TBT_ICON_SIZE, TBT_ICON_SIZE }, "ic_turn_r", 1.0f); break;
            case 3: ui_draw_image(s, { bx - TBT_ICON_SIZE / 2, by - TBT_ICON_SIZE / 2, TBT_ICON_SIZE, TBT_ICON_SIZE }, "ic_lane_change_l", 1.0f); break;
            case 4: ui_draw_image(s, { bx - TBT_ICON_SIZE / 2, by - TBT_ICON_SIZE / 2, TBT_ICON_SIZE, TBT_ICON_SIZE }, "ic_lane_change_r", 1.0f); break;
            case 7: ui_draw_image(s, { bx - TBT_ICON_SIZE / 2, by - TBT_ICON_SIZE / 2, TBT_ICON_SIZE, TBT_ICON_SIZE }, "ic_turn_u", 1.0f); break;
            case 6: ui_draw_text(s, bx, by + 15, "TG", FS(28), COLOR_WHITE, BOLD); break;
            case 8: ui_draw_text(s, bx, by + 15, "목적지", FS(26), COLOR_WHITE, BOLD); break;
            default:
                sprintf(str, "감속:%d", xTurnInfo);
                ui_draw_text(s, bx, by + 15, str, FS(26), COLOR_WHITE, BOLD);
                break;
            }
            if (s->scene.is_metric) {
              if (xDistToTurn < 1000) sprintf(str, "%d m", xDistToTurn);
              else  sprintf(str, "%.1f km", xDistToTurn / 1000.f);
            }
            else {
              if (xDistToTurn < 1609) sprintf(str, "%d ft", (int)(xDistToTurn * 3.28084));
              else sprintf(str, "%.1f mi", xDistToTurn / 1609.344f);
            }
            ui_draw_text(s, bx, by + 95, str, FS(32), COLOR_WHITE, BOLD);
        }

        // 우측: 도착예정시간 + 잔여거리 (세로 2줄, 폭 축소에 맞춰 폰트 40 -> 32)
        // [243차, 사용자 지시] 기존엔 1줄에 "도착: 37.1분(10:33)"을 전부 넣어
        // 폭 초과 시 시각(HH:MM)이 잘려 보이는 문제가 있었음. 잔여거리와
        // 표시 순서를 바꿔 1줄엔 "도착: 잔여거리"만, 2줄엔 잔여시간(분)+
        // 도착시각(HH:MM)을 배치 -> 2줄엔 라벨이 없어 폭 여유가 생겨 시각이
        // 잘리지 않음. 두 줄 전체를 한 줄(TBT_LINE_STEP=40)만큼 위로 이동해
        // 그 아래 도로명/route 블록과의 기존 간격(80px)은 그대로 유지.
        nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_BOTTOM);
        int info_x = tbt_x + 175;
        if (nGoPosDist > 0 && nGoPosTime > 0) {
            time_t now = time(NULL);  // 현재 시간 얻기
            struct tm* local = localtime(&now);
            int remaining_minutes = (int)nGoPosTime / 60;
            local->tm_min += remaining_minutes;
            mktime(local);
            bool is_kor = s->language == "main_ko";
            // 1줄: "도착 : 48.2km" (라벨 + 잔여거리)
            sprintf(str, "%s : %.1f%s", (is_kor)?"도착":"ETA", nGoPosDist / 1000. * ((s->scene.is_metric)?1:KM_TO_MILE), (s->scene.is_metric) ? "km" : "mile");
            ui_draw_text(s, info_x, tbt_y + 35, str, FS(30), COLOR_WHITE, BOLD);
            // 2줄: "37.1분(10:33)" (라벨 없이 잔여시간+도착시각만, 폭 여유 확보)
            sprintf(str, "%.1f%s(%02d:%02d)", (float)nGoPosTime / 60., (is_kor)?"분":"MIN", local->tm_hour, local->tm_min);
            ui_draw_text(s, info_x, tbt_y + 75, str, FS(30), COLOR_WHITE, BOLD);
        }

        // 하단: 과속카메라 안내(szSdiDescr) 또는 도로명(szPosRoadName), 전체 폭 사용
        // 128차: 127차 폭 축소(790->460)로 가용 폭이 420px로 줄어 긴 문구가 박스
        // 우측 경계를 넘어갈 수 있음 -> 폭 측정 후 초과 시 폰트를 FS(30)에서
        // 최소 FS(20)까지 3px 단위로 자동 축소(그래도 넘치면 FS(20) 유지, 잘림은
        // 감수). [231차] 기준값 자체를 TBT_FONT_SCALE(1.3x)만큼 키움.
        nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_BOTTOM);
        auto fit_bottom_text_size = [&](const char* txt) {
          int fs = FS(30);
          float bounds[4];
          float avail_w = TBT_BOX_W - 40; // 좌우 20px씩 여백
          while (fs > FS(20)) {
            nvgFontSize(s->vg, fs);
            nvgTextBounds(s->vg, tbt_x + 20, tbt_y + 220, txt, NULL, bounds);
            if (bounds[2] - bounds[0] <= avail_w) break;
            fs -= 3;
          }
          return fs;
        };
        if (szSdiDescr.length() > 0) {
            float bounds[4];  // [xmin, ymin, xmax, ymax]를 저장하는 배열
            int fs = fit_bottom_text_size(szSdiDescr.toStdString().c_str());
            nvgFontSize(s->vg, fs);
            nvgTextBounds(s->vg, tbt_x + 20, tbt_y + 220, szSdiDescr.toStdString().c_str(), NULL, bounds);
            float text_width = bounds[2] - bounds[0];
            float text_height = bounds[3] - bounds[1];
            ui_fill_rect(s->vg, { (int)bounds[0] - 8, (int)bounds[1] - 2, (int)text_width + 16, (int)text_height + 10 }, COLOR_GREEN, 8);
            ui_draw_text(s, tbt_x + 20, tbt_y + 220, szSdiDescr.toStdString().c_str(), fs, COLOR_WHITE, BOLD);
        }
        else if (szPosRoadName.length() > 0) {
          // 154차: 도로명(1줄) + route=NN.N(2줄)로 분리해서 그림(name_line1/name_line2는
          // 박스 크기 계산 시점에 이미 split·줄바꿈 계산 완료 — 위쪽 참고).
          // [230차, 사용자 지시] 두 줄 모두 박스 우측끝(right_x) 정렬로 변경.
          // 도로명이 길면 자동 줄바꿈(최대 TBT_MAX_NAME_LINES줄), 넘치는 만큼
          // 박스가 위로 커져 있어 전부 표시됨(그래도 넘치면 폰트 20까지 축소,
          // 최후에는 잘림 감수 — 128차 이후 기존 정책과 동일).

          // [243차, 사용자 지시] 도로명/route= 블록 전체를 한 줄(TBT_LINE_STEP=40)
          // 위로 이동(195->155). 비워진 기존 route 위치(235)는 아래 vturn=
          // 줄이 그대로 이어받아 박스 하단(tbt_y+280) 기준 여백은 변경 없음.
          // 1줄(들): 도로명, 우측끝 정렬. 마지막 줄이 tbt_y+155에 오도록 하고,
          // 추가된 줄은 그 위(TBT_LINE_STEP 간격)로 쌓는다.
          if (!name_wrapped.empty()) {
            nvgTextAlign(s->vg, NVG_ALIGN_RIGHT | NVG_ALIGN_BOTTOM);
            int n = (int)name_wrapped.size();
            for (int i = 0; i < n; i++) {
              int line_y = tbt_y + 155 - (n - 1 - i) * TBT_LINE_STEP;
              ui_draw_text(s, right_x, line_y, name_wrapped[i].c_str(), name_fs, COLOR_WHITE, BOLD);
            }
          }

          // 2줄: "route=" + 숫자(강조). prefix+숫자를 하나의 단위로 우측끝(right_x)에
          // 정렬(숫자 오른쪽 끝이 right_x, prefix가 그 왼쪽에 바로 이어 붙음 -> 화면상
          // "route=145.1" 순서, prefix가 왼쪽, 숫자가 오른쪽. 이 순서/로직은 변경 없음).
          // [232차, 사용자 지시] 154/156차 이후 유지되던 "숫자 = prefix의 2배" 비율을
          // 폐지. 숫자도 박스 안 다른 텍스트와 동일한 크기(FS(26))로 통일하고,
          // 강조는 색상(초록)만으로 유지한다.
          // [243차, 사용자 지시] y좌표 235->195로 한 줄 위로 이동(위 도로명 블록과 동일 폭).
          const int fs_prefix = FS(26);   // [231차] 전체 스케일(1.3x) 적용, route=/vturn= 공통 사용
          if (name_line2.length() > 0) {
            size_t eq = name_line2.find('=');
            std::string prefix = (eq == std::string::npos) ? name_line2 : name_line2.substr(0, eq + 1); // "route="
            std::string number = (eq == std::string::npos) ? "" : name_line2.substr(eq + 1);            // "145.1"

            const int fs_number = fs_prefix; // [232차] prefix와 동일 크기로 통일(기존 2배 비율 폐지), 색상만 초록으로 강조
            float line2_y = tbt_y + 195;      // [243차] 235->195 (한 줄 위로 이동)

            nvgTextAlign(s->vg, NVG_ALIGN_RIGHT | NVG_ALIGN_BOTTOM);
            float number_w = 0;
            if (number.length() > 0) {
              nvgFontSize(s->vg, fs_number);
              nvgFontFace(s->vg, BOLD);
              float bounds[4];
              nvgTextBounds(s->vg, 0, 0, number.c_str(), NULL, bounds);
              number_w = bounds[2] - bounds[0];
              ui_draw_text(s, right_x, line2_y, number.c_str(), fs_number, COLOR_GREEN, BOLD);
            }
            if (prefix.length() > 0) {
              ui_draw_text(s, right_x - number_w, line2_y, prefix.c_str(), fs_prefix, COLOR_WHITE, BOLD);
            }
          }

          // [243차, 사용자 지시 신규] route= 아래줄에 같은 크기(FS(26))/동일
          // 우측끝(right_x) 정렬로 "vturn=<값>" 추가. route= 줄이 비운 기존
          // y좌표(235)를 그대로 사용 -> 박스 하단 여백(45px) 기존과 동일.
          // vTurnSpeed는 carrotMan capnp 필드(cereal/custom.capnp)를 통해
          // carrot_serv.py에서 매 프레임 전달되는 실측값(단위: km/h로 취급).
          {
            char vturn_str[32];
            sprintf(vturn_str, "%d", nVTurnSpeed);
            std::string vt_prefix = "vturn=";
            float line3_y = tbt_y + 235; // route=가 235->195로 옮기며 비운 자리

            nvgTextAlign(s->vg, NVG_ALIGN_RIGHT | NVG_ALIGN_BOTTOM);
            nvgFontSize(s->vg, fs_prefix);
            nvgFontFace(s->vg, BOLD);
            float bounds[4];
            nvgTextBounds(s->vg, 0, 0, vturn_str, NULL, bounds);
            float vt_number_w = bounds[2] - bounds[0];
            ui_draw_text(s, right_x, line3_y, vturn_str, fs_prefix, COLOR_GREEN, BOLD);
            ui_draw_text(s, right_x - vt_number_w, line3_y, vt_prefix.c_str(), fs_prefix, COLOR_WHITE, BOLD);
          }
        }

        return 0;
    }
public:
    int draw(const UIState* s) {
        nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
        SubMaster& sm = *(s->sm);
        if (!sm.alive("modelV2") || !sm.alive("carrotMan") || !sm.alive("carState")) {
            active_carrot = -1;
            return -1;
        }
        const auto carrot_man = sm["carrotMan"].getCarrotMan();
          
        active_carrot = carrot_man.getActiveCarrot();

        if (active_carrot > 1) {
          xSpdLimit = carrot_man.getXSpdLimit();
          xSpdDist = carrot_man.getXSpdDist();
          xSignType = carrot_man.getXSpdType();
        }
        else {
          xSpdLimit = 0;
          xSpdDist = 0;
          xSignType = 0;
        }
        xTurnInfo = carrot_man.getXTurnInfo();
        xDistToTurn = carrot_man.getXDistToTurn();
        nRoadLimitSpeed = carrot_man.getNRoadLimitSpeed();
        nVTurnSpeed = carrot_man.getVTurnSpeed(); // [243차] TBT HUD vturn= 표시용
        if (active_carrot > 1 || carrot_man.getNGoPosDist() > 0) {
          atc_type = QString::fromStdString(carrot_man.getAtcType());

          nGoPosDist = carrot_man.getNGoPosDist();
          nGoPosTime = carrot_man.getNGoPosTime();
          szSdiDescr = QString::fromStdString(carrot_man.getSzSdiDescr());
          szPosRoadName = QString::fromStdString(carrot_man.getSzPosRoadName());
          szTBTMainText = QString::fromStdString(carrot_man.getSzTBTMainText());
          
        }
        else {
          //xTurnInfo = -1;
          //xDistToTurn = 0;
          //nRoadLimitSpeed = 20;
          atc_type = "";
          nGoPosDist = 0;
          nGoPosTime = 0;
          szSdiDescr = "";
          szPosRoadName = "";
          szTBTMainText = "";
        }

#ifdef __UI_TEST
        active_carrot = 2;
        xSpdLimit = 110;
        xSpdDist = 12345;
        nRoadLimitSpeed = 110;
#endif
        drawTurnInfo(s);
        drawSpeedLimit(s);
        return drawTurnInfoHud(s);

    }
};
bool _right_blinker = false;
bool _left_blinker = false;
class DesireDrawer : ModelDrawer {
protected:
    int icon_size = 256;
    int blinker_timer = 0;
public:
    void draw(const UIState* s, int x, int y) {
        blinker_timer = (blinker_timer + 1) % 16;
        nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
        SubMaster& sm = *(s->sm);
        if (!sm.alive("modelV2") || !sm.alive("carrotMan") || !sm.alive("carState")) return;
        auto meta = sm["modelV2"].getModelV2().getMeta();
        auto laneChangeDirection = meta.getLaneChangeDirection();
        auto laneChangeState = meta.getLaneChangeState();
        float desireStateTurnLeft = meta.getDesireState()[1];
        float desireStateTurnRight = meta.getDesireState()[2];
        float desireStateLaneChangeLeft = meta.getDesireState()[3];
        float desireStateLaneChangeRight = meta.getDesireState()[4];
        int desireEvent = 0;    // TODO

        if (desireStateTurnLeft > 0.5) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_turn_l", 1.0f);
        else if (desireStateTurnRight > 0.5) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_turn_r", 1.0f);
        else if (desireStateLaneChangeLeft > 0.5) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_l", 1.0f);
        else if (desireStateLaneChangeRight > 0.5) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_r", 1.0f);
        if (desireEvent == 57) {
            ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_steer", 1.0f);
            ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_l", 1.0f);
        }
        else if (desireEvent == 58) {
            ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_steer", 1.0f);
            ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_r", 1.0f);
        }
        else if (desireEvent == 71) {
            if (laneChangeDirection == cereal::LaneChangeDirection::LEFT) {
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_inhibit", 1.0f);
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_l", 1.0f);
            }
            else if (laneChangeDirection == cereal::LaneChangeDirection::RIGHT) {
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_inhibit", 1.0f);
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_r", 1.0f);
            }
        }
        if (laneChangeState == cereal::LaneChangeState::PRE_LANE_CHANGE) {
            if (laneChangeDirection == cereal::LaneChangeDirection::LEFT) {
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_steer", 1.0f);
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_l", 1.0f);
            }
            else if (laneChangeDirection == cereal::LaneChangeDirection::RIGHT) {
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_steer", 1.0f);
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_r", 1.0f);
            }
        }

        const auto carrot_man = sm["carrotMan"].getCarrotMan();
        const auto car_state = sm["carState"].getCarState();
        QString atc_type = QString::fromStdString(carrot_man.getAtcType());

        bool left_blinker = car_state.getLeftBlinker() || atc_type=="fork left" || atc_type =="turn left" || atc_type == "atc left";
        bool right_blinker = car_state.getRightBlinker() || atc_type=="fork right" || atc_type =="turn right" || atc_type == "atc right";

        _right_blinker = false;
        _left_blinker = false;
        if (blinker_timer <= 8) {
            if (right_blinker) {
		_right_blinker = true;
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_blinker_r", 1.0f);
            }
            if (left_blinker) {
		_left_blinker = true;
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_blinker_l", 1.0f);
            }
        }
    }
};

class BlindSpotDrawer : ModelDrawer{
protected:
    QPolygonF lane_barrier_vertices[2];

protected:
    void ui_draw_bsd(const UIState* s, const QPolygonF& vd, NVGcolor* color, bool right) {
        int index = vd.length();

        float x[4], y[4];
        for (int i = 0; i < index / 2 - 2; i += 2) {

            if (right) {
                x[0] = vd[i + 0].x();
                y[0] = vd[i + 0].y();
                x[1] = vd[i + 1].x();
                y[1] = vd[i + 1].y();
                x[2] = vd[index - i - 3].x();
                y[2] = vd[index - i - 3].y();
                x[3] = vd[index - i - 2].x();
                y[3] = vd[index - i - 2].y();
            }
            else {
                x[0] = vd[i + 0].x();
                y[0] = vd[i + 0].y();
                x[1] = vd[i + 1].x();
                y[1] = vd[i + 1].y();
                x[2] = vd[index - i - 3].x();
                y[2] = vd[index - i - 3].y();
                x[3] = vd[index - i - 2].x();
                y[3] = vd[index - i - 2].y();
            }
            ui_draw_line2(s, x, y, 4, color, nullptr, 3.0f);
        }
    }
    bool make_data(const UIState* s) {
        SubMaster& sm = *(s->sm);
        if (!sm.alive("modelV2")) return false;
        const cereal::ModelDataV2::Reader& model = sm["modelV2"].getModelV2();
        auto model_position = model.getPosition();
        int max_idx_barrier_l = get_path_length_idx(model_position, 40.0);
        int max_idx_barrier_r = get_path_length_idx(model_position, 40.0);
        update_line_data(s, model_position, 0, 1.2 - 0.05, 1.2 - 0.6, &lane_barrier_vertices[0], max_idx_barrier_l, false, -1.7); // 차선폭을 알면 좋겠지만...
        update_line_data(s, model_position, 0, 1.2 - 0.05, 1.2 - 0.6, &lane_barrier_vertices[1], max_idx_barrier_r, false, 1.7);
        return true;
    }
public:
    void draw(const UIState* s) {
        if (!make_data(s)) return;

        NVGcolor color = nvgRGBA(255, 215, 0, 150);
        NVGcolor color2 = nvgRGBA(0, 204, 0, 150);

        SubMaster& sm = *(s->sm);
        auto car_state = sm["carState"].getCarState();
        bool left_blindspot = car_state.getLeftBlindspot();
        bool right_blindspot = car_state.getRightBlindspot();

        auto lead_left = sm["radarState"].getRadarState().getLeadLeft();
        auto lead_right = sm["radarState"].getRadarState().getLeadRight();
        auto meta = sm["modelV2"].getModelV2().getMeta();
        auto laneChangeState = meta.getLaneChangeState();
        auto laneChangeDirection = meta.getLaneChangeDirection();
        bool rightLaneChange = (laneChangeState == cereal::LaneChangeState::PRE_LANE_CHANGE) &&
            (laneChangeDirection == cereal::LaneChangeDirection::RIGHT);
        bool leftLaneChange = (laneChangeState == cereal::LaneChangeState::PRE_LANE_CHANGE) &&
            (laneChangeDirection == cereal::LaneChangeDirection::LEFT);

#if 0
        left_blindspot = right_blindspot = true;
#endif
        if (left_blindspot) {
            ui_draw_bsd(s, lane_barrier_vertices[0], &color, false);
        }
        else if (lead_left.getStatus() && lead_left.getDRel() < car_state.getVEgo() * 3.0 && leftLaneChange) {
            ui_draw_bsd(s, lane_barrier_vertices[0], &color2, false);
        }

        if (right_blindspot) {
            ui_draw_bsd(s, lane_barrier_vertices[1], &color, true);
        }
        else if (lead_right.getStatus() && lead_right.getDRel() < car_state.getVEgo() * 3.0 && rightLaneChange) {
            ui_draw_bsd(s, lane_barrier_vertices[1], &color2, true);
        }
    }
};

class PathDrawer : ModelDrawer {
private:
    QPolygonF track_vertices;
    int     show_path_mode = 13;
    int     show_path_color = 14;
    int     show_path_mode_normal = 13;
    int     show_path_color_normal = 14;
    int     show_path_mode_lane = 13;
    int     show_path_color_lane = 14;
    int     show_path_color_cruise_off = 14;
    float   show_path_width = 1.0;
    Params  params;
    int     params_count = 0;
    bool    active_lane_line = false;

protected:
    void update_line_data2(const UIState* s, const cereal::XYZTData::Reader& line,
        float width_apply, float z_off_start, float z_off_end, QPolygonF* pvd, int max_idx, bool allow_invert = true) {
        const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();
        QPolygonF left_points, right_points;
        left_points.reserve(max_idx + 1);
        right_points.reserve(max_idx + 1);

        for (int i = 0; i <= max_idx; i++) {
            // highly negative x positions  are drawn above the frame and cause flickering, clip to zy plane of camera
            if (line_x[i] < 0) continue;
            float z_off = interp<float>((float)line_x[i], { 0.0f, 100.0 }, { z_off_start, z_off_end }, false);
            float y_off = interp<float>(z_off, { -3.0f, 0.0f, 3.0f }, { 1.5f, 0.5f, 1.5f }, false);
            y_off *= width_apply;

            QPointF left, right;
            bool l = _model->mapToScreen(line_x[i], line_y[i] - y_off, line_z[i] + z_off, &left);
            bool r = _model->mapToScreen(line_x[i], line_y[i] + y_off, line_z[i] + z_off, &right);
            if (l && r) {
                // For wider lines the drawn polygon will "invert" when going over a hill and cause artifacts
                if (!allow_invert && left_points.size() && left.y() > left_points.back().y()) {
                    continue;
                }
                left_points.push_back(left);
                right_points.push_front(right);
            }
        }
        *pvd = left_points + right_points;
    }
    void update_line_data_dist(const UIState* s, const cereal::XYZTData::Reader& line,
        float width_apply, float z_off_start, float z_off_end, QPolygonF* pvd, float max_dist, bool allow_invert = true) {
        const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();
        QPolygonF left_points, right_points;
        left_points.reserve(40 + 1);
        right_points.reserve(40 + 1);
        float idxs[line_x.size()], line_xs[line_x.size()], line_ys[line_x.size()], line_zs[line_x.size()];
        float   x_prev = 0;
        for (int i = 0; i < line_x.size(); i++) {
            idxs[i] = (float)i;
            if (i > 0 && line_x[i] < x_prev) {
                //printf("plan data error.\n");
                line_xs[i] = x_prev;
            }
            else line_xs[i] = line_x[i];
            x_prev = line_xs[i];
            line_ys[i] = line_y[i];
            line_zs[i] = line_z[i];
        }

        float   dist = 2.0;// , dist_dt = 1.;
        bool    exit = false;
        //printf("\ndist = ");
        for (; !exit; dist = dist + dist * 0.15) {
            //dist_dt += (i*0.05);
            if (dist >= max_dist) {
                dist = max_dist;
                exit = true;
            }
            //printf("%.0f ", dist);
            float z_off = interp<float>(dist, { 0.0f, 100.0 }, { z_off_start, z_off_end }, false);
            float y_off = interp<float>(z_off, { -3.0f, 0.0f, 3.0f }, { 1.5f, 0.5f, 1.5f }, false);
            y_off *= width_apply;
            float  idx = interp<float>(dist, line_xs, idxs, line_x.size(), false);
            if (idx >= line_x.size()) break;
            float line_y1 = interp<float>(idx, idxs, line_ys, line_x.size(), false);
            float line_z1 = interp<float>(idx, idxs, line_zs, line_x.size(), false);

            QPointF left, right;
            bool l = _model->mapToScreen(dist, line_y1 - y_off, line_z1 + z_off, &left);
            bool r = _model->mapToScreen(dist, line_y1 + y_off, line_z1 + z_off, &right);
            if (l && r) {
                // For wider lines the drawn polygon will "invert" when going over a hill and cause artifacts
                if (!allow_invert && left_points.size() && left.y() > left_points.back().y()) {
                    //printf("invert...\n");
                    continue;
                }
                left_points.push_back(left);
                right_points.push_front(right);
            }
            //else printf("range out..\n");
        }
        *pvd = left_points + right_points;
    }
    float dist_function(float t, float max_dist) {
        float dist = 3.0 * pow(1.2, t);
        return (dist >= max_dist) ? max_dist : dist;
    }
    float pos_t = 0.0f;
    void update_line_data_dist3(const UIState* s, const cereal::XYZTData::Reader& line,
        float width_apply, float z_off_start, float z_off_end, QPolygonF* pvd, float max_dist, bool allow_invert = true) {

        const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();
        QPolygonF left_points, right_points;
        left_points.reserve(41);
        right_points.reserve(41);

        // Prepare line data (correcting x-axis if needed)
        std::vector<float> idxs(line_x.size()), line_xs(line_x.size()), line_ys(line_x.size()), line_zs(line_x.size());
        float x_prev = 0;
        for (int i = 0; i < line_x.size(); i++) {
            idxs[i] = static_cast<float>(i);
            line_xs[i] = (i > 0 && line_x[i] < x_prev) ? x_prev : line_x[i];
            x_prev = line_xs[i];
            line_ys[i] = line_y[i];
            line_zs[i] = line_z[i];
        }

        // Get simulation state
        SubMaster& sm = *(s->sm);
        //auto controls_state = sm["controlsState"].getControlsState();
        auto car_state = sm["carState"].getCarState();
        float v_ego_kph = car_state.getVEgoCluster() * MS_TO_KPH;


        // Calculate time step and position
        //float pos_t = 0.0f;
        float dt = std::min(v_ego_kph * 0.01f, (show_path_mode >= 10) ? 0.6f : 1.0f);
        if (v_ego_kph < 1) pos_t = 4.0f;
        else if (dt < 0.2f) dt = 0.2f;
        pos_t += dt;
        if (pos_t > 24.0f) pos_t = pos_t - 24.0f;

        // Add time points based on path mode
        std::vector<float> draw_t;
        draw_t.reserve(50);
        draw_t.push_back(pos_t);

        auto add_time_points = [&](int count, float interval) {
            for (int i = 0; i < count; i++) {
                float t = draw_t.back();
                draw_t.push_back((t + interval > 24.0f) ? t + interval - 24.0f : t + interval);
            }
            };

        switch (show_path_mode) {
        case 9:
            add_time_points(1, 3.0f);
            add_time_points(1, 10.0f);
            add_time_points(1, 3.0f);
            break;
        case 10:
            add_time_points(7, 3.0f);
            break;
        case 11:
            add_time_points(5, 3.0f);
            break;
        case 12: {
            int n = std::clamp(static_cast<int>(v_ego_kph * 0.058f - 0.5f), 0, 7);
            add_time_points(n, 3.0f);
            break;
        }
        }

        // Find the smallest time value
        auto min_elem = std::min_element(draw_t.begin(), draw_t.end());
        int draw_t_idx = std::distance(draw_t.begin(), min_elem);

        // Process each time point and calculate points
        bool exit = false;
        for (int i = 0; i <= draw_t.size() && !exit; i++) {
            float t = draw_t[draw_t_idx];
            draw_t_idx = (draw_t_idx + 1) % draw_t.size();
            if (t < 3.0f) continue;

            float dist = dist_function(t, max_dist);
            if (dist == max_dist) exit = true;

            for (int j = 2; j >= 0; j--) {
                dist = exit ? dist_function(100, max_dist) : dist_function(t - j * 1.0f, max_dist);
                float z_off = interp<float>(dist, { 0.0f, 100.0f }, { z_off_start, z_off_end }, false);
                float y_off = interp<float>(z_off, { -3.0f, 0.0f, 3.0f }, { 1.5f, 0.5f, 1.5f }, false) * width_apply;
                float idx = interp<float>(dist, line_xs.data(), idxs.data(), line_x.size(), false);

                if (idx >= line_x.size()) {
                    printf("index... %.1f\n", idx);
                    break;
                }

                float line_y1 = interp<float>(idx, idxs.data(), line_ys.data(), line_x.size(), false);
                float line_z1 = interp<float>(idx, idxs.data(), line_zs.data(), line_x.size(), false);

                QPointF left, right;
                bool l = _model->mapToScreen(dist, line_y1 - y_off, line_z1 + z_off, &left);
                bool r = _model->mapToScreen(dist, line_y1 + y_off, line_z1 + z_off, &right);
                if (l && r) {
                    left_points.push_back(left);
                    right_points.push_front(right);
                }

                if (exit) break;
            }
        }

        // Combine left and right points into the polygon
        *pvd = left_points + right_points;
    }
protected:
    bool longActive = false;
    bool make_data(const UIState* s) {
		  SubMaster& sm = *(s->sm);
		  if (!sm.alive("modelV2") || !sm.alive("carState")) return false;
        const cereal::ModelDataV2::Reader& model = sm["modelV2"].getModelV2();
        active_lane_line = sm["controlsState"].getControlsState().getActiveLaneLine();
        auto model_position = model.getPosition();
        if (active_lane_line) {
          model_position = sm["lateralPlan"].getLateralPlan().getPosition();
        }
        float max_distance = s->max_distance;
        max_distance -= 2.0;
        int max_idx = get_path_length_idx(model_position, max_distance);

        auto selfdrive_state = sm["selfdriveState"].getSelfdriveState();
        longActive = selfdrive_state.getEnabled();
        if (active_lane_line) {
          show_path_mode = show_path_mode_lane;
          show_path_color = show_path_color_lane;
        }
        else {
          show_path_mode = show_path_mode_normal;
          show_path_color = show_path_color_normal;
        }
        if (longActive == false) {
            show_path_color = show_path_color_cruise_off;
        }

        if (show_path_mode == 0) {
            update_line_data2(s, model_position, show_path_width, 1.22, 1.22, &track_vertices, max_idx);
        }
        else if (show_path_mode < 9 || show_path_mode == 13 || show_path_mode == 14 || show_path_mode == 15) {
            update_line_data_dist(s, model_position, show_path_width, 1.22, 1.22, &track_vertices, max_distance, false);
        }
        else
            update_line_data_dist3(s, model_position, show_path_width, 1.22, 1.22, &track_vertices, max_distance, false);

        return true;
    }
    int use_lane_line_speed_apply = 0;
public:
    void draw(const UIState* s, float& pathDrawSeq) {
        SubMaster& sm = *(s->sm);
        auto car_state = sm["carState"].getCarState();
        params_count = (params_count + 1) % 20;
        if (params_count == 0) {
            show_path_mode_normal = params.getInt("ShowPathMode");
            show_path_color_normal = params.getInt("ShowPathColor");
            show_path_mode_lane = params.getInt("ShowPathModeLane");
            show_path_color_lane = params.getInt("ShowPathColorLane");
            show_path_color_cruise_off = params.getInt("ShowPathColorCruiseOff");
        }
        if (!make_data(s)) return;
        int temp = (int)car_state.getUseLaneLineSpeed();
        if (temp != use_lane_line_speed_apply) {
            ui_draw_text_a(s, 0, 0, (temp>0)?"LaneMode":"Laneless", 30, (temp>0)?COLOR_GREEN:COLOR_YELLOW, BOLD);
            use_lane_line_speed_apply = temp;
        }
        static bool forward = true;
        int alpha = 120;
        NVGcolor colors[10] = {
            COLOR_RED_ALPHA(alpha),           nvgRGBA(255, 153, 0, alpha),
            COLOR_YELLOW_ALPHA(alpha),        COLOR_GREEN_ALPHA(alpha),
            COLOR_BLUE_ALPHA(alpha),          nvgRGBA(0, 0, 128, alpha),
            nvgRGBA(0x8b, 0, 0xff, alpha),    COLOR_OCHRE_ALPHA(alpha),
            COLOR_WHITE_ALPHA(alpha),         COLOR_BLACK_ALPHA(alpha),
        };

        bool brake_valid = car_state.getBrakeLights();
        const auto radar_state = sm["radarState"].getRadarState();
        auto lead_one = radar_state.getLeadOne();
        auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
        //float desired_distance = lp.getDesiredDistance();
        float accel = lp.getAccels()[0];

        if (show_path_color >= 20) {
          if (longActive) {
            show_path_color = 13;// green
            if (lead_one.getStatus()) {
              if (abs(accel) < 0.5f) show_path_color = 12; // yellow
              else if (accel >= 0.5f) show_path_color = 11; // amber
              else show_path_color = 10; // red
            }
          }
          else {
            show_path_color = 19; // black
          }
        }

        if (show_path_mode == 0) {
            ui_draw_line(s, track_vertices, &colors[show_path_color % 10], nullptr,
                (show_path_color >= 10 || brake_valid) ? 2.0 : 0.0,
                brake_valid ? COLOR_RED : COLOR_WHITE);
        }
        else if (show_path_mode >= 13 && show_path_mode <= 15) {
            drawSpecialModes(s, show_path_mode, show_path_color, brake_valid, colors);
        }
        else if (show_path_mode >= 9) {
            drawComplexPath(s, show_path_color, brake_valid, colors);
        }
        else {
            drawAnimatedPath(s, car_state, pathDrawSeq, forward, show_path_mode, show_path_color, brake_valid, colors);
        }
    }

private:
    void drawSpecialModes(const UIState* s, int mode, int color_idx,
        bool brake_valid, NVGcolor* colors) {
        int track_vertices_len = track_vertices.length();
        float xp[3][128], yp[3][128];
        float g = 0.05f, gc = 0.4f;

        if (mode == 13) {
            g = 0.2f;
            gc = 0.10f;
        }
        else if (mode == 14) {
            g = 0.45f;
            gc = 0.05f;
        }
        else if (mode == 15) {
            gc = g;
        }

        int glen = track_vertices_len / 2 - 1;
        for (int i = 0; i < glen; i++) {
            int e = track_vertices_len - i - 1;
            int ge = glen * 2 - 1 - i;
            float x1 = track_vertices[i].x();
            float y1 = track_vertices[i].y();
            float x2 = track_vertices[e].x();
            float y2 = track_vertices[e].y();
            xp[0][i] = x1;
            yp[0][i] = y1;
            xp[0][ge] = x1 + (x2 - x1) * g;
            yp[0][ge] = y1 + (y2 - y1) * g;
            xp[1][i] = x1 + (x2 - x1) * (0.5f - gc);
            yp[1][i] = y1 + (y2 - y1) * (0.5f - gc);
            xp[1][ge] = x1 + (x2 - x1) * (0.5f + gc);
            yp[1][ge] = y1 + (y2 - y1) * (0.5f + gc);
            xp[2][i] = x1 + (x2 - x1) * (1.0f - g);
            yp[2][i] = y1 + (y2 - y1) * (1.0f - g);
            xp[2][ge] = x2;
            yp[2][ge] = y2;
        }
        if (mode == 13 || mode == 14) {
            ui_draw_line2(s, xp[0], yp[0], glen * 2, &colors[color_idx % 10], nullptr,
                (color_idx >= 10 || brake_valid) ? 2.0 : 0.0,
                brake_valid ? COLOR_RED : COLOR_WHITE);
        }
        if (mode == 13 || mode == 15) {
            ui_draw_line2(s, xp[1], yp[1], glen * 2, &colors[color_idx % 10], nullptr,
                (color_idx >= 10 || brake_valid) ? 2.0 : 0.0,
                brake_valid ? COLOR_RED : COLOR_WHITE);
        }
        if (mode == 13 || mode == 14) {
            ui_draw_line2(s, xp[2], yp[2], glen * 2, &colors[color_idx % 10], nullptr,
                (color_idx >= 10 || brake_valid) ? 2.0 : 0.0,
                brake_valid ? COLOR_RED : COLOR_WHITE);
        }
    }

    void drawComplexPath(const UIState* s, int color_idx, bool brake_valid,
        NVGcolor* colors) {
        int track_vertices_len = track_vertices.length();
        float x[6], y[6];
        int color_n = 0;
        for (int i = 0; i < track_vertices_len / 2 - 1; i += 3) {
            int e = track_vertices_len - i - 1;
            x[0] = track_vertices[i].x();
            y[0] = track_vertices[i].y();
            x[1] = track_vertices[i + 1].x();
            y[1] = track_vertices[i + 1].y();
            x[2] = (track_vertices[i + 2].x() + track_vertices[e - 2].x()) / 2;
            y[2] = (track_vertices[i + 2].y() + track_vertices[e - 2].y()) / 2;
            x[3] = track_vertices[e - 1].x();
            y[3] = track_vertices[e - 1].y();
            x[4] = track_vertices[e].x();
            y[4] = track_vertices[e].y();
            x[5] = (x[1] + x[3]) / 2;
            y[5] = (y[1] + y[3]) / 2;
            ui_draw_line2(s, x, y, 6, &colors[color_idx % 10], nullptr,
                (color_idx >= 10 || brake_valid) ? 2.0 : 0.0,
                brake_valid ? COLOR_RED : COLOR_WHITE);
            if (++color_n > 6) color_n = 0;
        }
    }

    void drawAnimatedPath(const UIState* s, const cereal::CarState::Reader& car_state,
        float& pathDrawSeq, bool& forward, int mode, int color_idx, bool brake_valid,
        NVGcolor* colors) {
        int track_vertices_len = track_vertices.length();
        float accel = car_state.getAEgo();
        float v_ego_kph = car_state.getVEgo() * MS_TO_KPH;

#ifdef __TEST
        v_ego_kph = 20.0f;
        accel = -1.2f;
#endif

        float seq = std::max(0.3f, v_ego_kph / 100.0f);
        if (accel < -1.0f) forward = false;
        if (accel > -0.5f) forward = true;
        int max_seq = std::min(track_vertices_len / 4 + 3, 16);

        static int pathDrawSeq2 = -1;

        if (forward) {
            pathDrawSeq += seq;
            if (pathDrawSeq > max_seq) {
                pathDrawSeq = (pathDrawSeq2 >= 0) ? pathDrawSeq2 : 0;
            }
        }
        else {
            pathDrawSeq -= seq;
            if (pathDrawSeq < 0) {
                pathDrawSeq = (pathDrawSeq2 >= 0) ? pathDrawSeq2 : max_seq;
            }
        }
        pathDrawSeq2 = (max_seq > 15) ? ((int)pathDrawSeq - max_seq / 2 + max_seq) % max_seq : -5;

        //float x[6], y[6];
        switch (mode) {
        case 1:
        case 2:
        case 5:
        case 6:
            drawMode1To6(s, pathDrawSeq, pathDrawSeq2, mode, color_idx, brake_valid, colors);
            break;
        case 3:
        case 4:
        case 7:
        case 8:
            drawMode3To8(s, pathDrawSeq, pathDrawSeq2, mode, color_idx, brake_valid, colors);
            break;
        }
    }

    void drawMode1To6(const UIState* s, float pathDrawSeq, int pathDrawSeq2, int mode,
        int color_idx, bool brake_valid, NVGcolor* colors) {
        int track_vertices_len = track_vertices.length();
        float x[4], y[4];
        for (int i = 0, color_n = 0; i < track_vertices_len / 2 - 4; i += 2) {
            x[0] = track_vertices[i].x();
            y[0] = track_vertices[i].y();
            x[1] = track_vertices[i + 2].x();
            y[1] = track_vertices[i + 2].y();
            x[2] = track_vertices[track_vertices_len - i - 3].x();
            y[2] = track_vertices[track_vertices_len - i - 3].y();
            x[3] = track_vertices[track_vertices_len - i - 1].x();
            y[3] = track_vertices[track_vertices_len - i - 1].y();

            bool draw = ((int)pathDrawSeq == i / 2 || (int)pathDrawSeq == i / 2 - 2 ||
                pathDrawSeq2 == i / 2 || pathDrawSeq2 == i / 2 - 2);
            if (track_vertices_len / 2 < 8 || mode == 5 || mode == 6) draw = true;

            if (draw) {
                int idx = (mode == 2 || mode == 6) ? color_n : color_idx % 10;
                ui_draw_line2(s, x, y, 4, &colors[idx], nullptr,
                    (color_idx >= 10 || brake_valid) ? 2.0 : 0.0,
                    brake_valid ? COLOR_RED : COLOR_WHITE);
            }

            if (i > 1) color_n = (color_n + 1) % 7;
        }
    }

    void drawMode3To8(const UIState* s, float pathDrawSeq, int pathDrawSeq2, int mode,
        int color_idx, bool brake_valid, NVGcolor* colors) {
        int track_vertices_len = track_vertices.length();
        float x[6], y[6];
        for (int i = 0, color_n = 0; i < track_vertices_len / 2 - 4; i += 2) {
            x[0] = track_vertices[i].x();
            y[0] = track_vertices[i].y();
            x[1] = track_vertices[i + 2].x();
            y[1] = track_vertices[i + 2].y();
            x[2] = (track_vertices[i + 4].x() + track_vertices[track_vertices_len - i - 5].x()) / 2;
            y[2] = (track_vertices[i + 4].y() + track_vertices[track_vertices_len - i - 5].y()) / 2;
            x[3] = track_vertices[track_vertices_len - i - 3].x();
            y[3] = track_vertices[track_vertices_len - i - 3].y();
            x[4] = track_vertices[track_vertices_len - i - 1].x();
            y[4] = track_vertices[track_vertices_len - i - 1].y();
            x[5] = (x[1] + x[3]) / 2;
            y[5] = (y[1] + y[3]) / 2;

            bool draw = ((int)pathDrawSeq == i / 2 || (int)pathDrawSeq == i / 2 - 2 ||
                pathDrawSeq2 == i / 2 || pathDrawSeq2 == i / 2 - 2);
            if (track_vertices_len / 2 < 8 || mode == 7 || mode == 8) draw = true;

            if (draw) {
                int idx = (mode == 4 || mode == 8) ? color_n : color_idx % 10;
                ui_draw_line2(s, x, y, 6, &colors[idx], nullptr,
                    (color_idx >= 10 || brake_valid) ? 2.0 : 0.0,
                    brake_valid ? COLOR_RED : COLOR_WHITE);
            }

            if (i > 1) color_n = (color_n + 1) % 7;
        }
    }
};

#if 0
typedef struct {
    float x, y, d, v, y_rel, v_lat, radar, model_prob, score;
} lead_vertex_data;
#endif

char    carrot_man_debug[256] = "";
class DrawCarrot : public QObject {
    Q_OBJECT

protected:
    Params	params;
    Params	params_memory{ "/dev/shm/params" };
    int     icon_size = 256;
public:
    DrawCarrot() {
    }
    float   v_cruise = 0.0;
    float   v_ego = 0.0;
    float   apply_speed = 250.0;
    QString apply_source = "";
    bool    latActive = false;
    bool    longActive = false;
    int     xState = 0;
    int     trafficState = 0;
    int     trafficState_carrot = 0;
    int     active_carrot = 0;
    float   cruiseTarget = 0.0;
    int     myDrivingMode = 1;

    QString szPosRoadName = "";
    int     nRoadLimitSpeed = 30;
    int     nGoPosDist = 0;
    int     xSpdLimit = 0;
    int     xSignType = -1;
    QPointF nav_path_vertex[150];
    QPointF nav_path_vertex_xy[150];
    int     nav_path_vertex_count = 0;
    bool    nav_path_display = false;

    //std::vector<lead_vertex_data> lead_vertices_side;

    bool updateState(UIState *s) {
        const SubMaster& sm = *(s->sm);
        const bool cs_alive = sm.alive("carState");
        const bool car_state_alive = sm.alive("carState");
        const bool car_control_alive = sm.alive("carControl");
        const bool carrot_man_alive = sm.alive("carrotMan");
        const bool lp_alive = sm.alive("longitudinalPlan");
        //const auto cs = sm["controlsState"].getControlsState();
        const auto car_state = sm["carState"].getCarState();
        const auto car_control = sm["carControl"].getCarControl();
        const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
        const auto carrot_man = sm["carrotMan"].getCarrotMan();
        const cereal::ModelDataV2::Reader& model = sm["modelV2"].getModelV2();
        const auto radar_state = sm["radarState"].getRadarState();
        auto lead_one = radar_state.getLeadOne();
        auto model_position = model.getPosition();
        const auto lane_lines = model.getLaneLines();
        nav_path_display = params.getInt("ShowRouteInfo");

        if (!cs_alive || !car_control_alive || !car_state_alive || !lp_alive) return false;
        auto selfdrive_state = sm["selfdriveState"].getSelfdriveState();
        longActive = selfdrive_state.getEnabled();
        latActive = car_control.getLatActive();

        v_cruise = car_state.getVCruiseCluster();
        v_ego = car_state.getVEgoCluster();
        if (carrot_man_alive) {
            active_carrot = carrot_man.getActiveCarrot();
            apply_speed = carrot_man.getDesiredSpeed();
            apply_source = QString::fromStdString(carrot_man.getDesiredSource());
            if (apply_speed >= v_cruise) apply_source = "";
            szPosRoadName = QString::fromStdString(carrot_man.getSzPosRoadName());
            nRoadLimitSpeed = carrot_man.getNRoadLimitSpeed();
            xSpdLimit = carrot_man.getXSpdLimit();
            xSignType = carrot_man.getXSpdType();
            nGoPosDist = carrot_man.getNGoPosDist();
            QString atcType = QString::fromStdString(carrot_man.getAtcType());
            trafficState_carrot = carrot_man.getTrafficState();
            const auto velocity = model.getVelocity();

            if (nav_path_display) {
              QString naviPaths = QString::fromStdString(carrot_man.getNaviPaths());
              QStringList pairs = naviPaths.split(";");
              nav_path_vertex_count = 0;
              int max_z = lane_lines[2].getZ().size();
              float z_offset = 0.0;
              foreach(const QString & pair, pairs) {
                QStringList xy = pair.split(",");  // ","로 x와 y 구분                
                if (xy.size() == 3) {
                  //printf("coords = x: %.1f, y: %.1f, d:%.1f\n", xy[0].toFloat(), xy[1].toFloat(), xy[2].toFloat());
                  float x = xy[0].toFloat();
                  float y = xy[1].toFloat();
                  float d = xy[2].toFloat();
                  int idx = get_path_length_idx(lane_lines[2], d);

                  if (idx >= max_z) z_offset -= 0.05;
                  nav_path_vertex_xy[nav_path_vertex_count] = QPointF(y, -x);
                  _model->mapToScreen((x < 3.0) ? 5.0 : x, y, lane_lines[2].getZ()[idx] + z_offset, &nav_path_vertex[nav_path_vertex_count++]);
                  if (nav_path_vertex_count >= 150) break;
                }
              }
            }
            auto meta = sm["modelV2"].getModelV2().getMeta();
            QString desireLog = QString::fromStdString(meta.getDesireLog());
            sprintf(carrot_man_debug, "%s, m_kph= %d, %dkm/h TBT(%d): %dm, CAM(%d): %dkm/h, %dm, ATC(%s), T(%d)",
                desireLog.toStdString().c_str(),
                (int)(velocity.getX()[32] * 3.6),
                carrot_man.getDesiredSpeed(),
                carrot_man.getXTurnInfo(),
                carrot_man.getXDistToTurn(),
                carrot_man.getXSpdType(),
                carrot_man.getXSpdLimit(),
                carrot_man.getXSpdDist(),
                atcType.toStdString().c_str(),
                carrot_man.getTrafficState());
        }
		    else {
          active_carrot = 0;
			    apply_speed = 250.0;
          apply_source = "";
          carrot_man_debug[0] = 0;
          szPosRoadName = "";
          nRoadLimitSpeed = 30;
          nGoPosDist = 0;
		    }

        xState = lp.getXState();
        trafficState = lp.getTrafficState();
        cruiseTarget = lp.getCruiseTarget();
        myDrivingMode = lp.getMyDrivingMode();

        s->max_distance = std::clamp(*(model_position.getX().end() - 1),
            MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE);

        if (lead_one.getStatus()) {
            const float lead_d = lead_one.getDRel();
            s->max_distance = std::clamp((float)lead_d, 0.0f, s->max_distance);
        }

#if 0
        lead_vertices_side.clear();
        for (auto const& rs : { radar_state.getLeadsLeft(), radar_state.getLeadsRight(), radar_state.getLeadsCenter() }) {
            for (auto const& l : rs) {
                lead_vertex_data vd;
                QPointF vtmp;
                float z = lane_lines[2].getZ()[get_path_length_idx(lane_lines[2], l.getDRel())];
                if (_model->mapToScreen(l.getDRel(), -l.getYRel(), z - 0.61, &vtmp)) {
                    vd.x = vtmp.x();
                    vd.y = vtmp.y();
                    //printf("(%.1f,%.1f,%.1f) -> (%.1f, %.1f)\n", l.getDRel(), -l.getYRel(), z, vd.x, vd.y);
                    vd.d = l.getDRel();
                    vd.v = l.getVLeadK();
                    vd.y_rel = l.getDPath();// l.getYRel();
                    vd.v_lat = l.getVLat();
                    vd.radar = l.getRadar();
                    vd.model_prob = l.getModelProb();
                    vd.score = l.getScore();
                    lead_vertices_side.push_back(vd);
                }
            }
        }
#endif
        return true;
	  }
    void drawRadarInfo(UIState* s) {
      char str[128];
      int show_radar_info = params.getInt("ShowRadarInfo");
      float radar_lat_factor = params.getFloat("RadarLatFactor") / 100;
      if (show_radar_info > 0) {
        const SubMaster& sm = *(s->sm);
        const auto radar_state = sm["radarState"].getRadarState();
        const cereal::ModelDataV2::Reader& model = sm["modelV2"].getModelV2();
        const auto lane_lines = model.getLaneLines();
        int wStr = 40;

        for (auto const& rs : { radar_state.getLeadsLeft(), radar_state.getLeadsRight(), radar_state.getLeadsCenter() }) {
          for (auto const& l : rs) {
            QPointF side, a_side;
            float x, y, z, ax, ay;
            float v, v_lat, y_rel;
            float t = radar_lat_factor;   // 예측 시간
            float model_prob = 0.0f;
            //float score = 0.0f;
            float dRel = l.getDRel();

            // 현재점 투영
            z = lane_lines[2].getZ()[get_path_length_idx(lane_lines[2], l.getDRel())] - 0.61f;
            if (dRel > 2.5 && _model->mapToScreen(dRel, -l.getYRel(), z, &side)) {
              x = side.x();
              y = side.y();

              v = l.getVLeadK();
              v_lat = l.getVLat();
              y_rel = l.getYRel();

              bool radar = l.getRadar();
              model_prob = l.getModelProb();
              //score = l.getScore();

              // 속도 크기/표시값 (v_ego 없이 간단 처리)
              float v_abs = std::sqrt(v * v + v_lat * v_lat);
              float v_sum = (v >= 0.f) ? v_abs : -v_abs;

              if (v_abs > 3.0f) {
                // 미래점(월드) 계산
                float a_dRel = dRel + v * t;
                if (a_dRel < 2.0f) a_dRel = 2.0f;
                float a_yRel = y_rel + v_lat * t;

                // 미래점 투영 (y는 기존과 동일하게 부호 반전)
                if (std::fabs(v) > 3.0f && _model->mapToScreen(a_dRel, -a_yRel, z, &a_side)) {  // (필요시 z를 a_dRel로 재계산 권장)
                  ax = a_side.x();
                  ay = a_side.y();

                  QPolygonF vertext;
                  vertext.push_back(side);
                  vertext.push_back(a_side);
                  ui_draw_line(s, vertext, nullptr, nullptr, 3.0, (v_sum > 0.f)? COLOR_GREEN: COLOR_RED);
                  nvgBeginPath(s->vg);
                  nvgCircle(s->vg, ax, ay, 10);
                  nvgFillColor(s->vg, (v_sum > 0.f) ? COLOR_GREEN : COLOR_RED);
                  nvgFill(s->vg);
                }

                // 속도 텍스트/박스
                sprintf(str, "%.0f", (s->scene.is_metric) ? v_sum * MS_TO_KPH : v_sum * MS_TO_MPH);
                wStr = 35 * (int)strlen(str);
                ui_fill_rect(s->vg,
                  { (int)(x - wStr / 2), (int)(y - 35), wStr, 42 },
                  (!radar) ? COLOR_BLUE : (model_prob == 0.01f) ? COLOR_GREEN : (v_sum > 0.f) ? COLOR_ORANGE : COLOR_RED,
                  15);
                ui_draw_text(s, x, y, str, 40, COLOR_WHITE, BOLD);

                if (show_radar_info >= 2) {
                  sprintf(str, "%.1f", y_rel);
                  ui_draw_text(s, x, y - 40, str, 30, COLOR_WHITE, BOLD);

                  sprintf(str, "%.1f", (s->scene.is_metric)? dRel : dRel * KM_TO_MILE);
                  ui_draw_text(s, x, y + 30, str, 30, COLOR_WHITE, BOLD);
                }
              }
              else if (show_radar_info >= 3) {
                strcpy(str, "*");
                ui_draw_text(s, x, y, str, 40, COLOR_WHITE, BOLD);
              }
            }
          }
        }
      }
    }
#if 0
    void drawRadarInfo_old(UIState* s) {
        char str[128];
        int show_radar_info = params.getInt("ShowRadarInfo");
        if (show_radar_info > 0) {
            int wStr = 40;
            for (auto const& vrd : lead_vertices_side) {
                auto [rx, ry, rd, rv, ry_rel, v_lat, radar, model_prob, score] = vrd;
                float v_abs = 0.0;
                float v_sum = 0.0;
                if (v_ego > 1.0) v_sum = v_abs = rv;
                else {
                  v_abs = sqrtf(rv * rv + v_lat * v_lat);
                  v_sum = (rv >= 0) ? v_abs : -v_abs;
                }

                if (v_sum < -1.0 || v_sum > 1.0) {
                    sprintf(str, "%.0f", (s->scene.is_metric)? v_sum * MS_TO_KPH : v_sum * MS_TO_MPH);
                    wStr = 35 * (strlen(str) + 0);
                    ui_fill_rect(s->vg, { (int)(rx - wStr / 2), (int)(ry - 35), wStr, 42 }, (!radar) ? COLOR_BLUE : (model_prob==0.01f) ? COLOR_ORANGE : (v_sum > 0.) ? COLOR_GREEN : COLOR_RED, 15);
                    ui_draw_text(s, rx, ry, str, 40, COLOR_WHITE, BOLD);
                    if (show_radar_info >= 2) {
                        sprintf(str, "%.1f", ry_rel);
                        ui_draw_text(s, rx, ry - 40, str, 30, COLOR_WHITE, BOLD);
                        sprintf(str, "%.3f", score);
                        //sprintf(str, "%.2f", rd);
                        ui_draw_text(s, rx, ry + 30, str, 30, COLOR_WHITE, BOLD);
                    }
                }
                else if (show_radar_info >= 3) {
                    strcpy(str, "*");
                    ui_draw_text(s, rx, ry, str, 40, COLOR_WHITE, BOLD);
                }
            }
        }

    }
#endif
    void drawDebug(UIState* s) {
        if (params.getInt("ShowDebugUI") > 1) {
            nvgTextAlign(s->vg, NVG_ALIGN_RIGHT | NVG_ALIGN_BOTTOM);
            ui_draw_text(s, s->fb_w, s->fb_h - 10, carrot_man_debug, 25, COLOR_WHITE, BOLD, 1.0f, 1.0f);
        }
    }
    void drawNaviPath(UIState* s) {
      if (!nav_path_display) return;
#if 0
        if (nav_path_vertex_count > 0) {
			nvgBeginPath(s->vg);
			nvgMoveTo(s->vg, nav_path_vertex[0].x(), nav_path_vertex[0].y());
			for (int i = 1; i < nav_path_vertex_count; i++) {
                float x = nav_path_vertex[i].x();
                float y = nav_path_vertex[i].y();
                if (isnan(x) || isnan(y)) continue;
                nvgLineTo(s->vg, x, y);
			}
			nvgStrokeColor(s->vg, COLOR_GREEN);
			nvgStrokeWidth(s->vg, 20.0f);
			nvgStroke(s->vg);
		}
#elif 1
        if (nav_path_vertex_count > 1) {
            for(int i = 1; i < nav_path_vertex_count; i++) {
                float x = nav_path_vertex[i].x();
                float y = nav_path_vertex[i].y();
                if(isnan(x) || isnan(y)) continue;
				nvgBeginPath(s->vg);
                nvgCircle(s->vg, x, y, 10);
                nvgFillColor(s->vg, COLOR_GREEN);
                nvgFill(s->vg);
			}
        }

#else
        if (nav_path_vertex_count) {
            nvgBeginPath(s->vg);
            int bx = s->fb_w - 400;
            int by = s->fb_h - 400;
            float scale = 1.0;
            nvgSave(s->vg);
            nvgScissor(s->vg, bx - 350, by - 250, 700, 300);
            nvgMoveTo(s->vg, bx + nav_path_vertex_xy[0].x() * scale, by + nav_path_vertex_xy[0].y() * scale);
            for (int i = 1; i < nav_path_vertex_count; i++) {
                float x = nav_path_vertex_xy[i].x();
                float y = nav_path_vertex_xy[i].y();
                if (isnan(x) || isnan(y)) continue;
                nvgLineTo(s->vg, bx + x * scale, by + y * scale);
            }
            nvgStrokeColor(s->vg, COLOR_GREEN);
            nvgStrokeWidth(s->vg, 10.0f);
            nvgStroke(s->vg);

            nvgRestore(s->vg);
        }
#endif
    }
    char    cruise_speed_last[32] = "";
    char    driving_mode_str_last[32] = "";
    int     gap_last = 0;
    int     blink_timer = 0;
    int     disp_timer = 0;
    float cpuTemp = 0.0f;
    float cpuUsage = 0.0f;
    int   memoryUsage = 0;
    float freeSpace = 0.0f;
    float voltage = 0.0f;
    void drawHud(UIState* s) {
        int show_device_state = params.getInt("ShowDeviceState");
        blink_timer = (blink_timer + 1) % 16;
        disp_timer = (disp_timer + 1) % 64; 
        nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);

        int x = 140;// 120;
        int y = s->fb_h - 500;// 300;// 410;

        int bx = x;
        int by = y + 270;
#ifdef __UI_TEST
        xSpdLimit = 50;
        xSignType = 1;
#endif
        bool cam_detected = false;
        if (xSpdLimit > 0 && xSignType != 22 && xSignType != 4) cam_detected = true;
        NVGcolor stroke_color = COLOR_WHITE;
        NVGcolor bg_color = (cam_detected && blink_timer > 8)?COLOR_RED_ALPHA(180):COLOR_BLACK_ALPHA(90);
        if (show_device_state > 0) {
          ui_fill_rect(s->vg, { bx - 120, by - 270, 475, 495 }, bg_color, 30, 2, &stroke_color);
        }
        else {
          ui_fill_rect(s->vg, { bx - 120, by - 270 + 140, 475, 495 - 140 }, bg_color, 30, 2, &stroke_color);
        }


        // draw traffic light
        int icon_red = icon_size;
        int icon_green = icon_size;
        bool red_light = trafficState == 1;
        bool green_light = trafficState == 2;

        if(trafficState_carrot == 1) {
			red_light = true;
            icon_red *= 1.5;
		}
		else if(trafficState_carrot == 2) {
			green_light = true;
            icon_green *= 1.5;
		}
        if (red_light) ui_draw_image(s, { x - icon_red / 2, y - icon_red / 2 + 270, icon_red, icon_red }, "ic_traffic_red", 1.0f);
        else if (green_light) ui_draw_image(s, { x - icon_green / 2, y - icon_green / 2 + 270, icon_green, icon_green }, "ic_traffic_green", 1.0f);

        // draw speed
        char speed[32];
        sprintf(speed, "%.0f", (s->scene.is_metric)? v_ego * MS_TO_KPH : v_ego * MS_TO_MPH);
        ui_draw_text(s, bx, by + 50, speed, 120, COLOR_WHITE, BOLD, 3.0f, 8.0f);
        ui_draw_image(s, { bx - 100, by - 60, 350, 150 }, "ic_speed_bg", 1.0f);

        // draw cruise speed
        char cruise_speed[32];
        int cruise_x = bx + 170;
        int cruise_y = by + 15;
        if(longActive) sprintf(cruise_speed, "%d", (int)((s->scene.is_metric)?v_cruise: v_cruise * KM_TO_MILE + 0.5));
		    else sprintf(cruise_speed, "--");
        if (strcmp(cruise_speed_last, cruise_speed) != 0) {
			    strcpy(cruise_speed_last, cruise_speed);
          if(strcmp(cruise_speed, "--"))
            ui_draw_text_a(s, cruise_x, cruise_y, cruise_speed, 60, COLOR_GREEN, BOLD);
		    }
        ui_draw_text(s, cruise_x, cruise_y, cruise_speed, 60, COLOR_GREEN, BOLD, 1.0, 5.0, COLOR_BLACK, COLOR_BLACK);

        // draw apply speed
        NVGcolor textColor = COLOR_GREEN;
        NVGcolor white_color = COLOR_WHITE;
        char apply_speed_str[32];
        int apply_x = bx + 250;
        int apply_y = by - 50;

        if (apply_source.length()) {
            sprintf(apply_speed_str, "%d", (int)((s->scene.is_metric)?apply_speed:apply_speed * KM_TO_MILE + 0.5));
            textColor = COLOR_OCHRE;    // apply speed가 작동되면... 색을 바꾸자.
            ui_draw_text(s, apply_x, apply_y, apply_speed_str, 50, textColor, BOLD, 1.0, 5.0, COLOR_BLACK, COLOR_BLACK);
            ui_draw_text(s, apply_x, apply_y - 50, apply_source.toStdString().c_str(), 30, textColor, BOLD, 1.0, 5.0, COLOR_BLACK, COLOR_BLACK);
        }
		    else if(abs(cruiseTarget - v_cruise) > 0.5) {
            sprintf(apply_speed_str, "%d", (int)((s->scene.is_metric)?cruiseTarget: cruiseTarget * KM_TO_MILE + 0.5));
			      ui_draw_text(s, apply_x, apply_y, apply_speed_str, 50, textColor, BOLD, 1.0, 5.0, COLOR_BLACK, COLOR_BLACK);
            ui_draw_text(s, apply_x, apply_y - 50, "eco", 30, textColor, BOLD, 1.0, 5.0, COLOR_BLACK, COLOR_BLACK);
		    }
        const SubMaster& sm = *(s->sm);

        // draw gap info
        char driving_mode_str[32] = "연비";
        int driving_mode = myDrivingMode;// params.getInt("MyDrivingMode");
        NVGcolor mode_color = COLOR_GREEN_ALPHA(210);
        NVGcolor text_color = COLOR_WHITE;
        switch (driving_mode) {
        case 1: strcpy(driving_mode_str, tr("ECO").toStdString().c_str()); mode_color = COLOR_GREEN_ALPHA(210);  break;
        case 2: strcpy(driving_mode_str, tr("SAFE").toStdString().c_str()); mode_color = COLOR_ORANGE_ALPHA(210);  text_color = COLOR_WHITE;  break;
        case 3: strcpy(driving_mode_str, tr("NORM").toStdString().c_str()); mode_color = COLOR_GREY_ALPHA(210);  text_color = COLOR_WHITE;  break;
        case 4: strcpy(driving_mode_str, tr("FAST").toStdString().c_str()); mode_color = COLOR_RED_ALPHA(210);  break;
        default: strcpy(driving_mode_str, tr("ERRM").toStdString().c_str()); break;
        }
        int dx = bx - 50;
        int dy = by + 175;
        ui_fill_rect(s->vg, { dx - 55, dy - 38, 110, 48 }, mode_color, 15, 2);
        ui_draw_text(s, dx, dy - 2, driving_mode_str, 32, text_color, BOLD);
        if (strcmp(driving_mode_str, driving_mode_str_last)) ui_draw_text_a(s, dx, dy, driving_mode_str, 30, COLOR_WHITE, BOLD);
        strcpy(driving_mode_str_last, driving_mode_str);

        auto gps = (s->ublox_avaliable) ? sm["gpsLocationExternal"].getGpsLocationExternal() : sm["gpsLocation"].getGpsLocation();
        if (gps.getHasFix()) {
          ui_draw_text(s, dx, dy - 45, "GPS", 30, COLOR_GREEN, BOLD);
        }

        char gap_str[32];
        int gap = params.getInt("LongitudinalPersonality") + 1;
        dx = bx + 220;
        dy = by + 77;
        sprintf(gap_str, "%d", gap);
        ui_draw_text(s, dx, dy, gap_str, 40, COLOR_WHITE, BOLD);
        if (gap_last != gap) ui_draw_text_a(s, dx, dy, gap_str, 40, COLOR_WHITE, BOLD);
        gap_last = gap;

        dx = bx + 300 - 30;
        dy = by + 175 + 10;// -38;
        //float ddx = 70 / 4.;
        float ddy = 80 / 4.;
#ifdef __UI_TEST
        gap = 3;
#endif
        for (int i = 0; i < gap; i++) {
            //ui_fill_rect(s->vg, { (int)(dx + i * ddx), (int)dy, (int)ddx - 2, 48 }, COLOR_GREEN_ALPHA(180), 4, 3);
            ui_fill_rect(s->vg, { (int)(dx), (int)(dy - ddy*(i+1) + 2), (int)70, (int)ddy-2}, COLOR_GREEN_ALPHA(210), 4, 3, &white_color);
        }

        char gear_str[32] = "R";
        dx = bx + 305;
        dy = by + 60;
        //const SubMaster& sm = *(s->sm);
        auto carState = sm["carState"].getCarState();
        if (carState.getGearShifter() == cereal::CarState::GearShifter::UNKNOWN) strcpy(gear_str, "U");
        else if (carState.getGearShifter() == cereal::CarState::GearShifter::PARK) strcpy(gear_str, "P");
        else if (carState.getGearShifter() == cereal::CarState::GearShifter::DRIVE) {
            if (carState.getGearStep() > 0)
      				sprintf(gear_str, "%d", carState.getGearStep());
		      	else
				      strcpy(gear_str, "D");
        }
        else if(carState.getGearShifter() == cereal::CarState::GearShifter::NEUTRAL) strcpy(gear_str, "N");
        else if (carState.getGearShifter() == cereal::CarState::GearShifter::REVERSE) strcpy(gear_str, "R");
        else if (carState.getGearShifter() == cereal::CarState::GearShifter::SPORT) strcpy(gear_str, "S");
        else if(carState.getGearShifter() == cereal::CarState::GearShifter::LOW) strcpy(gear_str, "L");
        else if (carState.getGearShifter() == cereal::CarState::GearShifter::BRAKE) strcpy(gear_str, "B");
        else if (carState.getGearShifter() == cereal::CarState::GearShifter::ECO) strcpy(gear_str, "E");
		else strcpy(gear_str, "M");

        ui_fill_rect(s->vg, { dx - 35, dy - 70, 70, 80 }, COLOR_GREEN_ALPHA(210), 15, 3, &white_color);
        ui_draw_text(s, dx, dy, gear_str, 70, COLOR_WHITE, BOLD);

        dx = bx + 200;
        dy = by + 175;
#ifdef __UI_TEST
        active_carrot = 2;
#endif
        if (active_carrot >= 2) {
            ui_fill_rect(s->vg, { dx - 55, dy - 38, 110, 48 }, COLOR_GREEN, 15, 2);
            ui_draw_text(s, dx, dy, "APN", 40, COLOR_WHITE, BOLD);
        }
        else if (active_carrot >= 1) {
            ui_fill_rect(s->vg, { dx - 55, dy - 38, 110, 48 }, COLOR_BLUE_ALPHA(210), 15, 2);
            ui_draw_text(s, dx, dy, "APM", 40, COLOR_WHITE, BOLD);
        }
        if (nav_path_vertex_count > 1) {
            ui_draw_text(s, dx, dy - 45, "ROUTE", 30, COLOR_WHITE, BOLD);
		}
#ifdef __UI_TEST
        active_carrot = 2;
        nRoadLimitSpeed = 30;
        xSpdLimit = 50;
        xSignType = 1;
#endif

        //if (active_carrot >= 2 || nGoPosDist > 0) {
        if (true) {
            dx = bx + 75;
            dy = by + 175;
            int disp_speed = 0;
            NVGcolor limit_color = COLOR_GREEN_ALPHA(210);
            if (xSpdLimit > 0 && xSignType != 22) {
                disp_speed = (int)(xSpdLimit * ((s->scene.is_metric)?1:KM_TO_MILE) + 0.5);
                limit_color = (blink_timer <= 8) ? COLOR_RED_ALPHA(210) : COLOR_YELLOW_ALPHA(210);
                ui_draw_text(s, dx, dy-45, "CAM", 30, COLOR_WHITE, BOLD);
            }
            else {
                disp_speed = nRoadLimitSpeed;
		            disp_speed = (int)(disp_speed * ((s->scene.is_metric)?1.0:KM_TO_MILE) + 0.5);
                limit_color = (v_ego * 3.6 > disp_speed + 2) ? COLOR_RED_ALPHA(210) : COLOR_WHITE_ALPHA(210);
                ui_draw_text(s, dx, dy - 45, "LIMIT", 30, COLOR_WHITE, BOLD);
            }

            ui_fill_rect(s->vg, { dx - 55, dy - 38, 110, 48 }, limit_color, 15, 2);
            ui_draw_text(s, dx, dy, QString::number(disp_speed).toStdString().c_str(), 40, COLOR_WHITE, BOLD);
        }

        if (show_device_state) {
            char str[128];
            dx = bx - 35;
            dy = by - 200;
            mode_color = COLOR_GREEN_ALPHA(190);
            ui_fill_rect(s->vg, { dx - 65, dy - 38, 130, 90 }, (cpuTemp>80 && blink_timer<=8)?COLOR_RED : mode_color, 15, 2);
            ui_draw_text(s, dx, dy-5, "CPU", 25, COLOR_WHITE, BOLD);
            sprintf(str, "%.0f\u00B0C", cpuTemp);
            ui_draw_text(s, dx, dy + 40, str, 40, COLOR_WHITE, BOLD);

            dx += 150;
            ui_fill_rect(s->vg, { dx - 65, dy - 38, 130, 90 }, (memoryUsage > 85 && blink_timer <= 8) ? COLOR_RED : mode_color, 15, 2);
            ui_draw_text(s, dx, dy-5, "MEM", 25, COLOR_WHITE, BOLD);
            sprintf(str, "%d%%", memoryUsage);
            ui_draw_text(s, dx, dy + 40, str, 40, COLOR_WHITE, BOLD);

            dx += 150;
            if (disp_timer < 32) {
              ui_fill_rect(s->vg, { dx - 65, dy - 38, 130, 90 }, mode_color, 15, 2);
              ui_draw_text(s, dx, dy - 5, "DISK", 25, COLOR_WHITE, BOLD);
              sprintf(str, "%.0f%%", 100 - freeSpace);
              ui_draw_text(s, dx, dy + 40, str, 40, COLOR_WHITE, BOLD);
            }
            else {
              ui_fill_rect(s->vg, { dx - 65, dy - 38, 130, 90 }, mode_color, 15, 2);
              ui_draw_text(s, dx, dy - 5, "VOLT", 25, COLOR_WHITE, BOLD);
              sprintf(str, "%.1fV", voltage);
              ui_draw_text(s, dx, dy + 40, str, 40, COLOR_WHITE, BOLD);
            }
        }
    }
    void drawDateTime(const UIState* s) {
        char str[128];
        // 시간표시 (클립영상 시간 확인용: YY-MM-DD(요일) 위 / HH:MM:SS 아래)
        // 155차: 날짜가 안 나오는 문제 발생 -> 파라미터(ShowDateTime)가 2(시간만)로
        // 설정되어 있던 것이 원인으로 확인됨(코드 버그 아님). 설정에서 1(Time/Date)로
        // 변경 후 정상 동작 확인. 아래는 파라미터값 그대로 존중하는 원래 로직으로 복원.
        int show_datetime = params.getInt("ShowDateTime");
        if (show_datetime) {
            time_t now = time(nullptr);
            struct tm* local = localtime(&now);

            int x = 170;// s->fb_w - 300;
            int y = 70;// 날짜 줄 기준선
            int nav_y = y;

            nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
            if (show_datetime == 1 || show_datetime == 3) {
                // 26-08-30(일) 형식 (연-월-일 + 요일)
                const char* weekdays_ko[] = { "일", "월", "화", "수", "목", "금", "토" };
                strftime(str, sizeof(str), "%y-%m-%d", local);
                int weekday_index = local->tm_wday; // tm_wday: 0=일, 1=월, ..., 6=토
                snprintf(str + strlen(str), sizeof(str) - strlen(str), "(%s)", weekdays_ko[weekday_index]);

                ui_draw_text(s, x, y, str, 45, COLOR_WHITE, BOLD, 3.0f, 8.0f);
                nav_y = y;
            }
            if (show_datetime == 1 || show_datetime == 2) {
                // 12:30:51 형식 (시:분:초) - 날짜 줄이 있으면 그 아래에 표시
                int time_y = (show_datetime == 1) ? y + 70 : y;
                strftime(str, sizeof(str), "%H:%M:%S", local);
                ui_draw_text(s, x, time_y, str, 65, COLOR_WHITE, BOLD, 3.0f, 8.0f);
                nav_y = time_y;
            }
            if (false && szPosRoadName.size() > 0) {
                nvgTextAlign(s->vg, NVG_ALIGN_RIGHT | NVG_ALIGN_BOTTOM);
                ui_draw_text(s, x, nav_y, szPosRoadName.toStdString().c_str(), 35, COLOR_WHITE, BOLD, 3.0f, 8.0f);
            }
        }
    }
    void drawConnInfo(const UIState* s) {
        int y = 10;
        int x = 30;
        if (false) {
            ui_draw_image(s, { x, y, 120, 54 }, "ic_scc2", 1.0f);
            x += 135;
        }
        if (active_carrot >= 2) {
            ui_draw_image(s, { x, y, 120, 54 }, "ic_apn", 1.0f);
            x += 135;
        }
        //else if (hda_speedLimit > 0 && hda_speedLimitDistance > 0) {
        //    ui_draw_image(s, { x, y, 120, 54 }, "ic_hda", 1.0f);
        //    x += 135;
        //}
        else if (active_carrot >= 1) {
            ui_draw_image(s, { x, y, 120, 54 }, "ic_apm", 1.0f);
            x += 135;
        }
        else if (false) {
            ui_draw_image(s, { x, y, 120, 54 }, "ic_hda", 1.0f);
            x += 135;
        }
        if (false) {
            ui_draw_image(s, { x, y, 240, 54 }, "ic_radartracks", 1.0f);
            x += (120 + 135);
        }
    }
    NVGcolor get_tpms_color(float tpms) {
        if (tpms < 5 || tpms > 60) // N/A
            return nvgRGBA(255, 255, 255, 220);
        if (tpms < 31)
            return nvgRGBA(255, 90, 90, 220);
        return nvgRGBA(255, 255, 255, 220);
    }

    const char* get_tpms_text(float tpms) {
        if (tpms < 5 || tpms > 60) return "  -";
        static char str[32];
        snprintf(str, sizeof(str), "%.0f", round(tpms));
        return str;
    }
    void drawTpms(const UIState* s) {
        nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
        SubMaster& sm = *(s->sm);
        auto car_state = sm["carState"].getCarState();

        int bx = (192 - 24) / 2 + UI_BORDER_SIZE + 10;
        int by = s->fb_h - 280 / 2 + 10;
        auto tpms = car_state.getTpms();
        float fl = tpms.getFl();
        float fr = tpms.getFr();
        float rl = tpms.getRl();
        float rr = tpms.getRr();
        ui_draw_text(s, bx - 90, by - 55, get_tpms_text(fl), 38, get_tpms_color(fl), BOLD);
        ui_draw_text(s, bx + 90, by - 55, get_tpms_text(fr), 38, get_tpms_color(fr), BOLD);
        ui_draw_text(s, bx - 90, by + 80, get_tpms_text(rl), 38, get_tpms_color(rl), BOLD);
        ui_draw_text(s, bx + 90, by + 80, get_tpms_text(rr), 38, get_tpms_color(rr), BOLD);
    }
    void drawTpms2(const UIState* s) {
        nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
        SubMaster& sm = *(s->sm);
        auto car_state = sm["carState"].getCarState();

        int bx = s->fb_w - 125;
        int by = 130;
        auto tpms = car_state.getTpms();
        float fl = tpms.getFl();
        float fr = tpms.getFr();
        float rl = tpms.getRl();
        float rr = tpms.getRr();
#ifdef __UI_TEST
        fl = fr = rl = rr = 29;
#endif
        int dw = 80;
        ui_draw_text(s, bx - dw, by - 55, get_tpms_text(fl), 40, get_tpms_color(fl), BOLD);
        ui_draw_text(s, bx + dw, by - 55, get_tpms_text(fr), 40, get_tpms_color(fr), BOLD);
        ui_draw_text(s, bx - dw, by + 70, get_tpms_text(rl), 40, get_tpms_color(rl), BOLD);
        ui_draw_text(s, bx + dw, by + 70, get_tpms_text(rr), 40, get_tpms_color(rr), BOLD);
    }
    void drawTpms3(const UIState* s) {
      nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
      SubMaster& sm = *(s->sm);
      auto car_state = sm["carState"].getCarState();

      int bx = s->fb_w - 125;
      int by = s->fb_h - 280 / 2 + 15;
      auto tpms = car_state.getTpms();
      float fl = tpms.getFl();
      float fr = tpms.getFr();
      float rl = tpms.getRl();
      float rr = tpms.getRr();
#ifdef __UI_TEST
      fl = fr = rl = rr = 29;
#endif
      int dw = 80;
      ui_draw_text(s, bx - dw, by - 55, get_tpms_text(fl), 40, get_tpms_color(fl), BOLD);
      ui_draw_text(s, bx + dw, by - 55, get_tpms_text(fr), 40, get_tpms_color(fr), BOLD);
      ui_draw_text(s, bx - dw, by + 70, get_tpms_text(rl), 40, get_tpms_color(rl), BOLD);
      ui_draw_text(s, bx + dw, by + 70, get_tpms_text(rr), 40, get_tpms_color(rr), BOLD);
    }
    void makeDeviceInfo(const UIState* s) {
        SubMaster& sm = *(s->sm);
        auto deviceState = sm["deviceState"].getDeviceState();
        freeSpace = deviceState.getFreeSpacePercent();
        memoryUsage = deviceState.getMemoryUsagePercent();
        const auto cpuTempC = deviceState.getCpuTempC();
        const auto cpuUsagePercent = deviceState.getCpuUsagePercent();
        int   size = sizeof(cpuTempC) / sizeof(cpuTempC[0]);
        if (size > 0) {
            for (int i = 0; i < size; i++) {
                cpuTemp += cpuTempC[i];
            }
            cpuTemp /= static_cast<float>(size);
        }
        size = sizeof(cpuUsagePercent) / sizeof(cpuUsagePercent[0]);
        if (size > 0) {
            int cpu_size = 0;
            for (cpu_size = 0; cpu_size < size; cpu_size++) {
                if (cpuUsagePercent[cpu_size] <= 0) break;
                cpuUsage += cpuUsagePercent[cpu_size];
            }
            if (cpu_size > 0) cpuUsage /= cpu_size;
        }

        auto peripheralState = sm["peripheralState"].getPeripheralState();
        voltage = peripheralState.getVoltage() / 1000.0;
    }
    void drawDeviceInfo(const UIState* s) {
#ifdef WSL2
        return;
#endif
        makeDeviceInfo(s);
        if (params.getInt("ShowDebugUI") == 0) return;

        nvgTextAlign(s->vg, NVG_ALIGN_RIGHT | NVG_ALIGN_TOP);
        QString str = "";
        str.sprintf("MEM:%d%% DISK:%.0f%% CPU:%.0f%%,%.0f\u00B0C", memoryUsage, freeSpace, cpuUsage, cpuTemp);
        NVGcolor top_right_color = (cpuTemp > 85.0 || memoryUsage > 85.0) ? COLOR_ORANGE : COLOR_WHITE;
		ui_draw_text(s, s->fb_w - 10, 2, str.toStdString().c_str(), 30, top_right_color, BOLD, 3.0f, 1.0f);
    }

};
#if 0
#include "msgq/visionipc/visionipc_server.h"
#include <QImage>
#include <QPainter>
#include <QPainterPath> // 추가

class MapRenderer {
public:
    MapRenderer() : image_data(WIDTH* HEIGHT * 4) { initialize(); }

    void initialize() {
        const int NUM_VIPC_BUFFERS = 4;

        vipc_server = std::make_unique<VisionIpcServer>("navd");
        vipc_server->create_buffers(VisionStreamType::VISION_STREAM_MAP, NUM_VIPC_BUFFERS, WIDTH, HEIGHT);
        vipc_server->start_listener();
    }

    void render(QPointF nav_path_vertex_xy[], int vertex_count) {
        // QImage 생성 및 초기화
        QImage image(WIDTH, HEIGHT, QImage::Format_RGBA8888); // RGBA 형식
        image.fill(Qt::black); // 검정 배경

        QPainter painter(&image);
        painter.setPen(QPen(Qt::white, 15)); // 경로 색상 및 두께 설정
        painter.setRenderHint(QPainter::Antialiasing, true);

        // 경로 그리기
        if (vertex_count > 2) {
            QPainterPath path;
            path.moveTo(nav_path_vertex_xy[0].x() * pixels_per_meter + WIDTH / 2.0,
                nav_path_vertex_xy[0].y() * pixels_per_meter + HEIGHT / 2.0);
            for (int i = 1; i < vertex_count; ++i) {
                path.lineTo(nav_path_vertex_xy[i].x() * pixels_per_meter + WIDTH / 2.0,
                    nav_path_vertex_xy[i].y() * pixels_per_meter + HEIGHT / 2.0);
            }
            painter.drawPath(path);
        }
        painter.end();

        // QImage 데이터를 NanoVG에 전달
        memcpy(image_data.data(), image.bits(), WIDTH * HEIGHT * 4); // RGBA 크기
    }

    bool publish() {
        double cur_t = millis_since_boot();
        if (cur_t > vipc_sent_t + 500) {
            vipc_sent_t = cur_t;

            // Vision IPC 버퍼에 데이터 전송
            VisionBuf* buf = vipc_server->get_buffer(VisionStreamType::VISION_STREAM_MAP);

            uint8_t* dst = (uint8_t*)buf->addr;
            memset(dst, 128, buf->len); // 초기화

            for (int i = 0; i < WIDTH * HEIGHT; i++) {
                dst[i] = image_data[i * 4]; // R 값을 사용하여 grayscale 변환
            }

            VisionIpcBufExtra extra = {
                .frame_id = frame_id,
                .timestamp_sof = nanos_since_boot(),
                .timestamp_eof = nanos_since_boot(),
                .valid = true
            };
            vipc_server->send(buf, &extra);
            frame_id++;
            return true;
        }
        return false;
    }

    void test_draw(NVGcontext* vg) {
        static int tex_id = -1; // 텍스처 ID 저장
        if (tex_id == -1) {
            tex_id = nvgCreateImageRGBA(vg, WIDTH, HEIGHT, 0, image_data.data());
        }
        else {
            // 텍스처 업데이트
            nvgUpdateImage(vg, tex_id, image_data.data());
        }

        // 텍스처를 화면에 그리기
        float x_offset = 200;
        float y_offset = 200;
        NVGpaint img_paint = nvgImagePattern(vg, x_offset, y_offset, WIDTH, HEIGHT, 0, tex_id, 1.0f);
        nvgBeginPath(vg);
        nvgRect(vg, x_offset, y_offset, WIDTH, HEIGHT); // 텍스처를 그릴 영역
        nvgFillPaint(vg, img_paint);
        nvgFill(vg);
    }

private:
    std::unique_ptr<VisionIpcServer> vipc_server;
    uint32_t frame_id = 0;
    double vipc_sent_t = 0.0;
    float pixels_per_meter = 0.5f; // 픽셀 단위 조정
    const int HEIGHT = 256, WIDTH = 256;
    std::vector<unsigned char> image_data;
};
#endif

DrawPlot drawPlot;
DrawCarrot drawCarrot;
BlindSpotDrawer drawBlindSpot;
PathDrawer drawPath;
LaneLineDrawer drawLaneLine;
PathEndDrawer drawPathEnd;
DesireDrawer drawDesire;
TurnInfoDrawer drawTurnInfo;

OnroadAlerts::Alert alert;
NVGcolor alert_color;

//MapRenderer mapRenderer;

void ui_draw(UIState *s, ModelRenderer* model_renderer, int w, int h) {
  _model = model_renderer;
  if (s->fb_w != w || s->fb_h != h) {
    ui_resize(s, w, h);
  }
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  nvgBeginFrame(s->vg, s->fb_w, s->fb_h, 1.0f);
  nvgScissor(s->vg, 0, 0, s->fb_w, s->fb_h);

  //nvgFontSize(s->vg, 170);
  //nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
  //ui_draw_text(s, 500, 500, "Carrot", 100, COLOR_GREEN, BOLD);
  Params params;
  bool draw_carrot = drawCarrot.updateState(s);
  drawCarrot.drawNaviPath(s);
  static float pathDrawSeq = 0.0;
  int show_lane_info = params.getInt("ShowLaneInfo");
  if(show_lane_info >= 0) drawPath.draw(s, pathDrawSeq);
  drawLaneLine.draw(s, show_lane_info);
  if (params.getInt("ShowPathEnd") > 0) drawPathEnd.draw(s);

  int path_x = drawPathEnd.getPathX();
  int path_y = drawPathEnd.getPathY();
  drawDesire.draw(s, path_x, path_y - 135);
  

  drawPlot.draw(s);

  drawBlindSpot.draw(s);

  if(draw_carrot)
    drawCarrot.drawRadarInfo(s);

  drawCarrot.drawHud(s);

  drawCarrot.drawDebug(s);
  drawCarrot.drawDateTime(s);
  //drawCarrot.drawConnInfo(s);
  drawCarrot.drawDeviceInfo(s);
  int show_tpms = params.getInt("ShowTpms");
  switch (show_tpms) {
  case 0: break;
  case 1:
    drawCarrot.drawTpms2(s);
    break;
  case 2:
    drawCarrot.drawTpms3(s);
    break;
  case 3:
    drawCarrot.drawTpms2(s);
    drawCarrot.drawTpms3(s);
    break;
  }

  drawTurnInfo.draw(s);

  ui_draw_text_a2(s);
  ui_draw_alert(s);

#if 0
  if (drawCarrot.nav_path_vertex_count > 1) {
      mapRenderer.render(drawCarrot.nav_path_vertex_xy, drawCarrot.nav_path_vertex_count);
      mapRenderer.publish();
      mapRenderer.test_draw(s->vg);
  }
#endif
  nvgResetScissor(s->vg);
  nvgEndFrame(s->vg);
  glDisable(GL_BLEND);
}

class BorderDrawer {
protected:
    float   a_ego_width = 0.0;
    float steering_angle_pos = 0.0;
    NVGcolor get_tpms_color(float tpms) {
        if (tpms < 5 || tpms > 60) // N/A
            return COLOR_GREEN;
        if (tpms < 31)
            return COLOR_RED;
        return COLOR_GREEN;
    }

    const char* get_tpms_text(float tpms) {
        if (tpms < 5 || tpms > 60) return "-";
        static char str[32];
        snprintf(str, sizeof(str), "%.0f", round(tpms));
        return str;
    }
public:
    void drawTpms(UIState* s, int w, int h) {
        NVGcontext* vg = s->vg_border;
        nvgTextAlign(vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
        SubMaster& sm = *(s->sm);
        if (!sm.alive("carState")) return;
        auto car_state = sm["carState"].getCarState();

        auto tpms = car_state.getTpms();
        float fl = tpms.getFl();
        float fr = tpms.getFr();
        float rl = tpms.getRl();
        float rr = tpms.getRr();

        //fl = fr = rl = rr = 29;
        //fl = fr = 40;
        ui_draw_text_vg(vg, 25, 30, get_tpms_text(fl), 30, get_tpms_color(fl), BOLD, 3);
        ui_draw_text_vg(vg, w - 25, 30, get_tpms_text(fr), 30, get_tpms_color(fr), BOLD, 3);
        ui_draw_text_vg(vg, 25, h, get_tpms_text(rl), 30, get_tpms_color(rl), BOLD, 3);
        ui_draw_text_vg(vg, w - 25, h, get_tpms_text(rr), 30, get_tpms_color(rr), BOLD, 3);
    }
    void draw(UIState *s, int w, int h, NVGcolor bg, NVGcolor bg_long) {
        NVGcontext* vg = s->vg_border;

        ui_fill_rect(vg, { 0,0, w, h / 2  - 100}, bg, 15);
        ui_fill_rect(vg, { 0, h / 2 + 100, w, h }, bg_long, 15);

        ui_fill_rect(vg, {w - 50, h/2 - 95, 50, 190}, (_right_blinker)?COLOR_ORANGE:COLOR_BLACK, 15);
        ui_fill_rect(vg, {0, h/2 - 95, 50, 190}, (_left_blinker)?COLOR_ORANGE:COLOR_BLACK, 15);

        const SubMaster& sm = *(s->sm);
        auto car_state = sm["carState"].getCarState();
        float a_ego = car_state.getAEgo();

        a_ego_width = a_ego_width * 0.5 + (w * std::abs(a_ego) / 4.0) * 0.5;
        ui_fill_rect(vg, { w/2 - (int)(a_ego_width / 2), h - 30, (int)a_ego_width, 30 }, (a_ego >= 0)? COLOR_YELLOW : COLOR_RED, 15);

        steering_angle_pos = steering_angle_pos * 0.5 + (w / 2. - w / 2. * car_state.getSteeringAngleDeg() / 90) * 0.5;
        int x_st = (int)steering_angle_pos - 50;
        int x_ed = (int)steering_angle_pos + 50;
        if (x_st < 0) x_st = 0;
        if (x_ed < 50) x_ed = 50;
        if (x_st > w - 50) x_st = w - 50;
        if (x_ed > w) x_ed = w;
        ui_fill_rect(vg, { x_st, 0, x_ed - x_st, 30 }, COLOR_ORANGE, 15);


        char top[256] = "", top_left[256] = "", top_right[256] = "";
        char bottom[256] = "", bottom_left[256] = "", bottom_right[256] = "";

        Params params = Params();
        QString str;

        // top
        str = QString::fromStdString(car_state.getLogCarrot());
        sprintf(top, "%s", str.toStdString().c_str());
        // top_right
        const auto live_delay = sm["liveDelay"].getLiveDelay();
        const auto live_torque_params = sm["liveTorqueParameters"].getLiveTorqueParameters();
        const auto live_params = sm["liveParameters"].getLiveParameters();
        str.sprintf("LD[%.0f%%,%.2f],LT[%.0f%%,%s](%.2f/%.2f), SR(%.1f,%.1f)",
            (float)live_delay.getCalPerc(), live_delay.getLateralDelay(),
            (float)live_torque_params.getCalPerc(), live_torque_params.getLiveValid() ? "ON" : "OFF",
            live_torque_params.getLatAccelFactorFiltered(), live_torque_params.getFrictionCoefficientFiltered(),
            live_params.getSteerRatio(), params.getFloat("CustomSR")/10.0);
        sprintf(top_right, "%s", str.toStdString().c_str());

        //top_left
        QString carName = QString::fromStdString(params.get("CarName"));
        bool longitudinal_control = sm["carParams"].getCarParams().getOpenpilotLongitudinalControl();
        if (params.getInt("HyundaiCameraSCC") > 0) {
            carName += "(CAMERA SCC)";
        }
        else if (longitudinal_control) {
            carName += " - OP Long";
        }
        QString NNFFModelName = QString::fromStdString(params.get("NNFFModelName"));
        if (NNFFModelName.length() > 0) {
            carName += ",NNFF";
        }
        sprintf(top_left, "%s", carName.toStdString().c_str());

        // bottom
        const auto lat_plan = sm["lateralPlan"].getLateralPlan();
        str = lat_plan.getLatDebugText().cStr();
        strcpy(bottom, str.toStdString().c_str());

        // bottom_left
        QString gitBranch = QString::fromStdString(params.get("GitBranch"));
        QString drivingModelName = QString::fromStdString(params.get("DrivingModelName"));
        if (drivingModelName.isEmpty()) {
            drivingModelName = DEFAULT_MODEL_NAME;
        }
        sprintf(bottom_left, "%s (%s)", gitBranch.toStdString().c_str(), drivingModelName.toStdString().c_str());

        // bottom_right
        Params params_memory = Params("/dev/shm/params");
        if (false && carrot_man_debug[0] != 0 && params.getInt("ShowDebugUI") > 0) {
            strcpy(bottom_right, carrot_man_debug);
        }
        else {
            QString ipAddress = QString::fromStdString(params_memory.get("NetworkAddress"));
            //extern QString gitBranch;
            sprintf(bottom_right, "%s", ipAddress.toStdString().c_str());
        }

        int text_margin = 30;
        // top
        nvgTextAlign(vg, NVG_ALIGN_CENTER | NVG_ALIGN_TOP);
        ui_draw_text_vg(vg, w / 2, 0, top, 30, COLOR_WHITE, BOLD);
        // top left
        nvgTextAlign(vg, NVG_ALIGN_LEFT | NVG_ALIGN_TOP);
        ui_draw_text_vg(vg, text_margin, 0, top_left, 30, COLOR_WHITE, BOLD);
        // top right
        nvgTextAlign(vg, NVG_ALIGN_RIGHT | NVG_ALIGN_TOP);
        ui_draw_text_vg(vg, w - text_margin, 0, top_right, 30, COLOR_WHITE, BOLD);
        // bottom
        nvgTextAlign(vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
        ui_draw_text_vg(vg, w / 2, h, bottom, 30, COLOR_WHITE, BOLD);
        // bottom left
        nvgTextAlign(vg, NVG_ALIGN_LEFT | NVG_ALIGN_BOTTOM);
        ui_draw_text_vg(vg, text_margin, h, bottom_left, 30, COLOR_WHITE, BOLD);
        // bottom right
        nvgTextAlign(vg, NVG_ALIGN_RIGHT | NVG_ALIGN_BOTTOM);
        ui_draw_text_vg(vg, w- text_margin, h, bottom_right, 30, COLOR_WHITE, BOLD);

        //drawTpms(s, w, h);
    }
};
NVGcolor QColorToNVGcolor(const QColor& color) {
    return nvgRGBAf(color.redF(), color.greenF(), color.blueF(), color.alphaF());
}
BorderDrawer borderDrawer;
void ui_draw_border(UIState* s, int w, int h, QColor bg, QColor bg_long) {

    if (s->vg_border == nullptr) {
        s->vg_border = nvgCreate(NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG);
        std::pair<const char*, const char*> fonts[] = {
            {"KaiGenGothicKR-Bold", "../assets/addon/font/KaiGenGothicKR-Bold.ttf"},
        };
        for (auto [name, file] : fonts) {
            int font_id = nvgCreateFont(s->vg_border, name, file);
            assert(font_id >= 0);
        }
    }
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glViewport(0, 0, w, h);
    nvgBeginFrame(s->vg_border, w, h, 1.0f);
	nvgScissor(s->vg_border, 0, 0, w, h);

    borderDrawer.draw(s, w, h, QColorToNVGcolor(bg), QColorToNVGcolor(bg_long));

    nvgResetScissor(s->vg_border);
    nvgEndFrame(s->vg_border);
    glDisable(GL_BLEND);
}
void ui_draw_alert(UIState* s) {
    if (alert.size != cereal::SelfdriveState::AlertSize::NONE) {
        alert_color = COLOR_ORANGE;
        nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
        ui_draw_text(s, s->fb_w / 2, s->fb_h - 300, alert.text1.toStdString().c_str(), 100, alert_color, BOLD, 3.0f, 8.0f);
        ui_draw_text(s, s->fb_w / 2, s->fb_h - 200, alert.text2.toStdString().c_str(), 70, alert_color, BOLD, 3.0f, 8.0f);
    }
}
void ui_update_alert(const OnroadAlerts::Alert& a) {
    //alert_color = nvgRGBA(color.red(), color.green(), color.blue(), color.alpha());
    //printf("r=%d, g=%d, b=%d\n", color.red(), color.green(), color.blue());
    alert = a;
}

void ui_nvg_init(UIState *s) {
  // on EON, we enable MSAA
  //s->vg = Hardware::EON() ? nvgCreate(0) : nvgCreate(NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG);
  s->vg = nvgCreate(NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG);
  assert(s->vg);

  // init fonts
  std::pair<const char *, const char *> fonts[] = {
      //{"sans-regular", "../assets/fonts/opensans_regular.ttf"},
      //{"sans-semibold", "../assets/fonts/opensans_semibold.ttf"},
      //{"sans-bold", "../assets/fonts/opensans_bold.ttf"},
      //{"KaiGenGothicKR-Normal", "../assets/addon/font/KaiGenGothicKR-Normal.ttf"},
      //{"KaiGenGothicKR-Medium", "../assets/addon/font/KaiGenGothicKR-Medium.ttf"},
      {"KaiGenGothicKR-Bold", "../assets/addon/font/KaiGenGothicKR-Bold.ttf"},
      //{"Inter-Bold", "../assets/fonts/Inter-Bold.ttf"},
  };
  for (auto [name, file] : fonts) {
    int font_id = nvgCreateFont(s->vg, name, file);
    assert(font_id >= 0);
  }
  // init images
  std::vector<std::pair<const char *, const char *>> images = {
  {"ic_lane_change_l", "../assets/images/lane_change_l.png"},
  {"ic_lane_change_r", "../assets/images/lane_change_r.png"},
  {"ic_lane_change_inhibit", "../assets/images/lane_change_inhibit.png"},
  {"ic_lane_change_steer", "../assets/images/lane_change_steer.png"},
  {"ic_bsd_l", "../assets/images/bsd_l.png"},
  {"ic_bsd_r", "../assets/images/bsd_r.png"},
  {"ic_turn_l", "../assets/images/turn_l.png"},
  {"ic_turn_r", "../assets/images/turn_r.png"},
  {"ic_turn_u", "../assets/images/turn_u.png"},
  {"ic_blinker_l", "../assets/images/blink_l.png"},
  {"ic_blinker_r", "../assets/images/blink_r.png"},
  {"ic_speed_bg", "../assets/images/speed_bg.png"},
  {"ic_traffic_green", "../assets/images/traffic_green.png"},
  {"ic_traffic_red", "../assets/images/traffic_red.png"},
  {"ic_tire", "../assets/images/img_tire.png"},
  {"ic_road_speed", "../assets/images/road_speed.png"},
  {"ic_speed_bump", "../assets/images/speed_bump.png"},
  {"ic_nda", "../assets/images/img_nda.png"},
  {"ic_navi","../assets/images/img_navi.png"},
  {"ic_scc2", "../assets/images/img_scc2.png"},
  {"ic_radartracks", "../assets/images/img_radartracks.png"},
  {"ic_apm", "../assets/images/img_apm.png"},
  {"ic_apn", "../assets/images/img_apn.png"},
  {"ic_hda", "../assets/images/img_hda.png"},
  {"ic_navi_point", "../assets/images/navi_point.png"}

  };
  for (auto [name, file] : images) {
    s->images[name] = nvgCreateImage(s->vg, file, 1);
    assert(s->images[name] != 0);
  }
}
void ui_resize(UIState *s, int width, int height) {
  s->fb_w = width;
  s->fb_h = height;
}

#include "selfdrive/ui/carrot.moc"
