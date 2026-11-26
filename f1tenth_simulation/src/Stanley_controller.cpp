/** ----------------------------------------------------------------------------
 * @file: stanley_controller_ackermann_node.cpp
 * @date: May 2026
 * @brief: Stanley Controller adaptado para simulador Ackermann.
 *         Solo regula el steering angle (fiel al diseño original).
 *         Velocidad constante configurable por parámetro.
 *
 * Suscribe:
 *   /r1/odom       (nav_msgs/Odometry)   — pose y velocidad del robot
 *   /drawn_plan    (nav_msgs/Path)        — trayectoria a seguir
 *
 * Publica:
 *   /r1/cmd_vel    (geometry_msgs/Twist)  — delta (angular.z), v_const (linear.x)
 *   /CTE           (std_msgs/Float32)     — crosstrack error (debug/métricas)
 *   /e_psi         (std_msgs/Float32)     — error de heading (debug/métricas)
 * -----------------------------------------------------------------------------
 **/

#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Implementación inline del controlador Stanley
// (Si tienes stanley_controller.hpp en tu proyecto, reemplaza por #include)
// ---------------------------------------------------------------------------
class StanleyController
{
public:
    double delta_{0.0};   // Steering angle calculado [rad]
    double ex_{0.0};      // Crosstrack error [m]
    double e_psi_{0.0};   // Error de heading [rad]

    StanleyController(double delta_max, double delta_min, double k, double k_soft)
        : k_(k), k_soft_(k_soft), delta_max_(delta_max), delta_min_(delta_min) {}

    /**
     * Calcula el crosstrack error firmado (+ = vehículo a la izquierda del path).
     */
    void calculateCrosstrackError(double vx, double vy,
                                  double p1x, double p1y,
                                  double p2x, double p2y)
    {
        double dx = p2x - p1x;
        double dy = p2y - p1y;
        double seg_len = std::hypot(dx, dy);

        if (seg_len < 1e-6) { ex_ = 0.0; return; }

        // Normal izquierda del segmento
        double nx = -dy / seg_len;
        double ny =  dx / seg_len;

        ex_ = (vx - p1x) * nx + (vy - p1y) * ny;
    }

    void setYawAngle(double psi) { psi_ = psi; }

    /**
     * Ley de Stanley:
     *   delta = e_psi + arctan( k * ex / (k_soft + v) )
     */
    void calculateSteering(double v, double path_yaw)
    {
        e_psi_ = std::atan2(std::sin(psi_ - path_yaw),
                            std::cos(psi_ - path_yaw));

        double delta_cte = std::atan2(k_ * ex_, k_soft_ + std::max(v, 0.1));

        delta_ = std::clamp(e_psi_ + delta_cte, delta_min_, delta_max_);
    }

private:
    double psi_{0.0};
    double k_;
    double k_soft_;
    double delta_max_;
    double delta_min_;
};

// ---------------------------------------------------------------------------
// Nodo ROS 2
// ---------------------------------------------------------------------------
class StanleyAckermannNode : public rclcpp::Node
{
public:
    StanleyAckermannNode() : Node("stanley_ackermann_node")
    {
        // ── Parámetros ────────────────────────────────────────────────────
        declare_parameter("K",         8.0);
        declare_parameter("K_soft",    1.1);
        declare_parameter("delta_max", 0.6);   // [rad] — igual que MPC
        declare_parameter("v_const",   0.5);   // [m/s] — velocidad fija

        double k         = get_parameter("K").as_double();
        double k_soft    = get_parameter("K_soft").as_double();
        double delta_max = get_parameter("delta_max").as_double();
        v_const_         = get_parameter("v_const").as_double();

        stanley_ = std::make_unique<StanleyController>(
            delta_max, -delta_max, k, k_soft);

        // ── Suscriptores ──────────────────────────────────────────────────
        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/r1/odom", 10,
            [this](const nav_msgs::msg::Odometry &msg) { odom_cb(msg); });

        sub_path_ = create_subscription<nav_msgs::msg::Path>(
            "/drawn_plan", 10,
            [this](const nav_msgs::msg::Path &msg) { path_cb(msg); });

        // ── Publicadores ──────────────────────────────────────────────────
        pub_cmd_  = create_publisher<geometry_msgs::msg::Twist>("/r1/cmd_vel", 10);
        pub_cte_  = create_publisher<std_msgs::msg::Float32>("/CTE", 10);
        pub_epsi_ = create_publisher<std_msgs::msg::Float32>("/e_psi", 10);

        // ── Servicio de inicio (mismo que el MPC) ─────────────────────────
        srv_start_ = create_service<std_srvs::srv::Empty>(
            "/start_execution",
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                         std::shared_ptr<std_srvs::srv::Empty::Response>) {
                if (!path_.empty()) {
                    executing_ = true;
                    RCLCPP_INFO(get_logger(),
                        "Stanley: ejecución iniciada. v_const=%.2f m/s", v_const_);
                }
            });

        // ── Timer @ 10 Hz (igual que el MPC) ─────────────────────────────
        timer_ = create_wall_timer(100ms,
            std::bind(&StanleyAckermannNode::control_loop, this));

        RCLCPP_INFO(get_logger(),
            "StanleyAckermannNode listo. Esperando /start_execution y /drawn_plan");
    }

private:
    // ── Estado del robot ───────────────────────────────────────────────────
    double x_robot_{0.0}, y_robot_{0.0}, yaw_robot_{0.0}, v_current_{0.0};

    // ── Trayectoria ────────────────────────────────────────────────────────
    std::vector<std::pair<double,double>> path_;
    size_t target_idx_{0};
    bool executing_{false};

    // ── Velocidad constante ────────────────────────────────────────────────
    // Se usa: (1) como cmd.linear.x publicado, (2) en el denominador de Stanley.
    // NO se adapta. Es el único parámetro de velocidad del controlador.
    double v_const_{0.5};

    // ── Controlador ───────────────────────────────────────────────────────
    std::unique_ptr<StanleyController> stanley_;

    // ── ROS handles ───────────────────────────────────────────────────────
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr      sub_path_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   pub_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr      pub_cte_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr      pub_epsi_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr          srv_start_;
    rclcpp::TimerBase::SharedPtr                              timer_;

    // ── Callbacks ─────────────────────────────────────────────────────────
    void odom_cb(const nav_msgs::msg::Odometry &msg)
    {
        x_robot_   = msg.pose.pose.position.x;
        y_robot_   = msg.pose.pose.position.y;
        v_current_ = msg.twist.twist.linear.x;

        const auto &q = msg.pose.pose.orientation;
        yaw_robot_ = std::atan2(2.0*(q.w*q.z + q.x*q.y),
                                1.0 - 2.0*(q.y*q.y + q.z*q.z));
    }

    void path_cb(const nav_msgs::msg::Path &msg)
    {
        path_.clear();
        for (const auto &pose : msg.poses)
            path_.emplace_back(pose.pose.position.x, pose.pose.position.y);
        target_idx_ = 0;
        RCLCPP_INFO(get_logger(), "Nueva trayectoria: %zu puntos", path_.size());
    }

    // ── Waypoint más cercano (ventana = 20, igual que el MPC) ─────────────
    void update_target_idx()
    {
        constexpr size_t WINDOW = 20;
        size_t end = std::min(target_idx_ + WINDOW, path_.size());
        double best = std::numeric_limits<double>::max();

        for (size_t i = target_idx_; i < end; ++i) {
            double d = std::hypot(x_robot_ - path_[i].first,
                                  y_robot_ - path_[i].second);
            if (d < best) { best = d; target_idx_ = i; }
        }
    }

    // ── Loop de control ───────────────────────────────────────────────────
    void control_loop()
    {
        if (!executing_ || path_.empty()) return;

        update_target_idx();

        // Fin de trayectoria
        if (target_idx_ >= path_.size() - 1) {
            RCLCPP_INFO(get_logger(), "Trayectoria completada.");
            executing_ = false;
            pub_cmd_->publish(geometry_msgs::msg::Twist{});
            return;
        }

        // Segmento actual p1 → p2
        double p1x = path_[target_idx_].first,    p1y = path_[target_idx_].second;
        double p2x = path_[target_idx_+1].first,   p2y = path_[target_idx_+1].second;
        double path_yaw = std::atan2(p2y - p1y, p2x - p1x);

        // Stanley: calcula SOLO el steering angle
        stanley_->calculateCrosstrackError(x_robot_, y_robot_, p1x, p1y, p2x, p2y);
        stanley_->setYawAngle(yaw_robot_);
        stanley_->calculateSteering(v_const_, path_yaw);

        // Publicar: steering calculado + velocidad CONSTANTE sin modificar
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = v_const_;            // ← fija, el controlador no la toca
        cmd.angular.z = -stanley_->delta_;    // ← único output real del Stanley
        pub_cmd_->publish(cmd);

        // Métricas (mismos topics que el MPC para comparación directa)
        std_msgs::msg::Float32 cte_msg, epsi_msg;
        cte_msg.data  = static_cast<float>(stanley_->ex_);
        epsi_msg.data = static_cast<float>(stanley_->e_psi_);
        pub_cte_->publish(cte_msg);
        pub_epsi_->publish(epsi_msg);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "wp=%zu | CTE=%.3f m | e_psi=%.3f rad | delta=%.3f rad | v=%.2f m/s (const)",
            target_idx_, stanley_->ex_, stanley_->e_psi_, stanley_->delta_, v_const_);
    }
};

// ---------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StanleyAckermannNode>());
    rclcpp::shutdown();
    return 0;
}