#include "stages/racetrack.h"
#include <cmath>
#include <queue>
#include <algorithm>
#include <limits>
#include <sstream>

namespace racetrack {

static const double EPS = 1e-12;
static const double PI  = 3.1415926535897932384626433832795;

static inline double deg2rad(double d){ return d * PI / 180.0; }
static inline double rad2deg(double r){ return r * 180.0 / PI; }

static inline Vec2 operator+(const Vec2& a, const Vec2& b){ return {a.x+b.x, a.y+b.y}; }
static inline Vec2 operator-(const Vec2& a, const Vec2& b){ return {a.x-b.x, a.y-b.y}; }
static inline Vec2 operator*(const Vec2& a, double k){ return {a.x*k, a.y*k}; }

static inline double dot(const Vec2& a, const Vec2& b){ return a.x*b.x + a.y*b.y; }
static inline double cross(const Vec2& a, const Vec2& b){ return a.x*b.y - a.y*b.x; }
static inline double norm2(const Vec2& a){ return dot(a,a); }
static inline double norm(const Vec2& a){ return std::sqrt(norm2(a)); }

struct Seg { Vec2 a,b; };

static inline double clamp01(double t){ return t<0?0:(t>1?1:t); }

static double dist_point_seg(const Vec2& p, const Seg& s){
  Vec2 ab = s.b - s.a;
  double denom = norm2(ab);
  if (denom < EPS) return norm(p - s.a);
  double t = dot(p - s.a, ab) / denom;
  t = clamp01(t);
  Vec2 proj = s.a + ab*t;
  return norm(p - proj);
}

static bool point_in_poly(const std::vector<Vec2>& poly, const Vec2& p){
  // ray casting; boundary treated as inside
  bool inside = false;
  int n = (int)poly.size();
  for(int i=0,j=n-1;i<n;j=i++){
    Vec2 a=poly[i], b=poly[j];
    if (dist_point_seg(p, Seg{a,b}) < 1e-10) return true;
    bool cond = (a.y > p.y) != (b.y > p.y);
    if (cond){
      double xint = (b.x-a.x)*(p.y-a.y)/(b.y-a.y) + a.x;
      if (xint > p.x) inside = !inside;
    }
  }
  return inside;
}

static int orient(const Vec2& a,const Vec2& b,const Vec2& c){
  double v = cross(b-a, c-a);
  if (std::fabs(v) < 1e-12) return 0;
  return (v>0)?1:-1;
}

static bool onseg(const Vec2& a,const Vec2& b,const Vec2& p){
  return std::min(a.x,b.x)-1e-12 <= p.x && p.x <= std::max(a.x,b.x)+1e-12 &&
         std::min(a.y,b.y)-1e-12 <= p.y && p.y <= std::max(a.y,b.y)+1e-12 &&
         std::fabs(cross(b-a, p-a)) < 1e-10;
}

static bool seg_intersect(const Seg& A, const Seg& B){
  Vec2 a=A.a,b=A.b,c=B.a,d=B.b;
  int o1=orient(a,b,c), o2=orient(a,b,d), o3=orient(c,d,a), o4=orient(c,d,b);
  if (o1==0 && onseg(a,b,c)) return true;
  if (o2==0 && onseg(a,b,d)) return true;
  if (o3==0 && onseg(c,d,a)) return true;
  if (o4==0 && onseg(c,d,b)) return true;
  return (o1*o2<0 && o3*o4<0);
}

static double dist_seg_seg(const Seg& s1, const Seg& s2){
  if (seg_intersect(s1,s2)) return 0.0;
  double d1 = dist_point_seg(s1.a, s2);
  double d2 = dist_point_seg(s1.b, s2);
  double d3 = dist_point_seg(s2.a, s1);
  double d4 = dist_point_seg(s2.b, s1);
  return std::min(std::min(d1,d2), std::min(d3,d4));
}

static bool segment_intersects_poly_edges(const std::vector<Vec2>& poly, const Seg& s){
  int n=(int)poly.size();
  for(int i=0;i<n;i++){
    Seg e{poly[i], poly[(i+1)%n]};
    if (seg_intersect(s,e)) return true;
  }
  return false;
}

static double min_dist_seg_to_poly_edges(const std::vector<Vec2>& poly, const Seg& s){
  int n=(int)poly.size();
  double best = std::numeric_limits<double>::infinity();
  for(int i=0;i<n;i++){
    Seg e{poly[i], poly[(i+1)%n]};
    best = std::min(best, dist_seg_seg(s,e));
  }
  return best;
}

static double signed_dist_to_poly(const std::vector<Vec2>& poly, const Vec2& p){
  double d = std::numeric_limits<double>::infinity();
  int n=(int)poly.size();
  for(int i=0;i<n;i++){
    d = std::min(d, dist_point_seg(p, Seg{poly[i], poly[(i+1)%n]}));
  }
  return point_in_poly(poly,p) ? d : -d;
}

struct Cell {
  double x,y,h,d,maxd;
};

static Vec2 polylabel_center(const std::vector<Vec2>& poly, double precision_m){
  double minx=1e100,miny=1e100,maxx=-1e100,maxy=-1e100;
  for(auto &p: poly){
    minx=std::min(minx,p.x); miny=std::min(miny,p.y);
    maxx=std::max(maxx,p.x); maxy=std::max(maxy,p.y);
  }
  double w=maxx-minx, h=maxy-miny;
  double cellSize = std::max(w,h);
  double step = cellSize/4.0;
  step = std::max(step, 1.0); // 或者 std::max(step, precision_m*2)
  if (step < 1.0) step = cellSize/2.0;

  auto make_cell = [&](double cx,double cy,double hh){
    Cell c;
    c.x=cx; c.y=cy; c.h=hh;
    c.d = signed_dist_to_poly(poly, Vec2{cx,cy});
    c.maxd = c.d + c.h*std::sqrt(2.0);
    return c;
  };

  struct Cmp{ bool operator()(const Cell& a, const Cell& b) const { return a.maxd < b.maxd; } };
  std::priority_queue<Cell, std::vector<Cell>, Cmp> pq;

  for(double x=minx; x<=maxx; x+=step){
    for(double y=miny; y<=maxy; y+=step){
      pq.push(make_cell(x+step/2.0, y+step/2.0, step/2.0));
    }
  }

  Vec2 best{(minx+maxx)/2.0, (miny+maxy)/2.0};
  double bestd = signed_dist_to_poly(poly,best);
  if (bestd < 0 && !pq.empty()){
    best = Vec2{pq.top().x, pq.top().y};
    bestd = pq.top().d;
  }

  int iter=0, maxIter=6000;
  while(!pq.empty() && iter++<maxIter){
    Cell c = pq.top(); pq.pop();
    if (c.d > bestd){
      bestd = c.d;
      best = Vec2{c.x,c.y};
    }
    if (c.maxd - bestd <= precision_m) continue;
    double hh = c.h/2.0;
    pq.push(make_cell(c.x-hh, c.y-hh, hh));
    pq.push(make_cell(c.x+hh, c.y-hh, hh));
    pq.push(make_cell(c.x-hh, c.y+hh, hh));
    pq.push(make_cell(c.x+hh, c.y+hh, hh));
  }
  return best;
}

static bool ray_segment_intersect(const Vec2& ro, const Vec2& rd, const Seg& s, double &t_out){
  // ro + t*rd intersects segment s.a + u*(s.b-s.a)
  Vec2 v1 = ro - s.a;
  Vec2 v2 = s.b - s.a;
  Vec2 v3{-rd.y, rd.x};
  double denom = dot(v2, v3);
  if (std::fabs(denom) < 1e-12) return false;
  double t = cross(v2, v1) / denom;
  double u = dot(v1, v3) / denom;
  if (t > 1e-10 && u > -1e-12 && u < 1.0+1e-12){
    t_out = t;
    return true;
  }
  return false;
}

static double ray_to_poly_exit_dist(const std::vector<Vec2>& poly, const Vec2& ro, const Vec2& rd){
  double best = std::numeric_limits<double>::infinity();
  int n=(int)poly.size();
  for(int i=0;i<n;i++){
    Seg e{poly[i], poly[(i+1)%n]};
    double t;
    if (ray_segment_intersect(ro, rd, e, t)) best = std::min(best, t);
  }
  return best;
}

struct Rect {
  Vec2 c;
  double theta=0; // rad
  double a=0;     // half-length along u
  double b=0;     // half-width along v
  double area() const { return 4.0*a*b; }
};

static std::vector<Vec2> rect_corners(const Rect& r){
  Vec2 u{std::cos(r.theta), std::sin(r.theta)};
  Vec2 v{-std::sin(r.theta), std::cos(r.theta)};
  std::vector<Vec2> pts;
  pts.push_back(r.c + u*r.a + v*r.b);
  pts.push_back(r.c + u*r.a - v*r.b);
  pts.push_back(r.c - u*r.a - v*r.b);
  pts.push_back(r.c - u*r.a + v*r.b);
  return pts;
}

static bool rect_inside_poly(const std::vector<Vec2>& poly, const Rect& r){
  auto pts = rect_corners(r);
  for(auto &p: pts) if(!point_in_poly(poly,p)) return false;
  for(int i=0;i<4;i++){
    Seg e{pts[i], pts[(i+1)%4]};
    if (segment_intersects_poly_edges(poly, e)){
      Vec2 mid = (e.a + e.b)*0.5;
      if (!point_in_poly(poly, mid)) return false;
    }
  }
  return true;
}

static Rect max_inscribed_rect_approx(const std::vector<Vec2>& poly, const Vec2& center, double theta){
  Vec2 u{std::cos(theta), std::sin(theta)};
  Vec2 v{-std::sin(theta), std::cos(theta)};

  double du = std::min(ray_to_poly_exit_dist(poly, center, u),
                       ray_to_poly_exit_dist(poly, center, u*(-1)));
  double dv = std::min(ray_to_poly_exit_dist(poly, center, v),
                       ray_to_poly_exit_dist(poly, center, v*(-1)));
  if (!std::isfinite(du) || !std::isfinite(dv)) du = dv = 0;

  Rect r{center, theta, std::max(0.0, du*0.95), std::max(0.0, dv*0.95)};

  auto bisect_max_a = [&](double b_fixed){
    double lo=0, hi=r.a;
    for(int it=0;it<35;it++){
      double mid=(lo+hi)/2;
      Rect t{center, theta, mid, b_fixed};
      if (rect_inside_poly(poly,t)) lo=mid; else hi=mid;
    }
    return lo;
  };
  auto bisect_max_b = [&](double a_fixed){
    double lo=0, hi=r.b;
    for(int it=0;it<35;it++){
      double mid=(lo+hi)/2;
      Rect t{center, theta, a_fixed, mid};
      if (rect_inside_poly(poly,t)) lo=mid; else hi=mid;
    }
    return lo;
  };

  for(int k=0;k<5;k++){
    r.a = bisect_max_a(r.b);
    r.b = bisect_max_b(r.a);
  }
  return r;
}

// Halton
static double halton(int index, int base){
  double f=1.0, r=0.0;
  while(index>0){
    f/=base;
    r += f*(index%base);
    index/=base;
  }
  return r;
}

// Solar azimuth (north=0 cw), then convert to east=0 ccw
static double solar_azimuth_north_cw(double lat_rad, double lon_rad, std::int64_t unix_utc){
  double jd = (double)unix_utc / 86400.0 + 2440587.5;
  double T = (jd - 2451545.0)/36525.0;

  double L0 = std::fmod(280.46646 + T*(36000.76983 + T*0.0003032), 360.0);
  if(L0<0) L0+=360.0;

  double M  = 357.52911 + T*(35999.05029 - 0.0001537*T);
  double e  = 0.016708634 - T*(0.000042037 + 0.0000001267*T);

  auto sind = [](double d){ return std::sin(d*PI/180.0); };
  auto cosd = [](double d){ return std::cos(d*PI/180.0); };

  double C = (1.914602 - T*(0.004817 + 0.000014*T))*sind(M)
           + (0.019993 - 0.000101*T)*sind(2*M)
           + 0.000289*sind(3*M);

  double trueLong = L0 + C;
  double omega = 125.04 - 1934.136*T;
  double lambda = trueLong - 0.00569 - 0.00478*sind(omega);

  double eps0 = 23.439291 - 0.0130042*T;
  double eps  = eps0 + 0.00256*cosd(omega);

  double decl = std::asin( std::sin(eps*PI/180.0) * std::sin(lambda*PI/180.0) );

  double y = std::tan((eps*PI/180.0)/2.0); y *= y;
  double Etime = 4.0*( y*std::sin(2*L0*PI/180.0) - 2*e*std::sin(M*PI/180.0)
                     + 4*e*y*std::sin(M*PI/180.0)*std::cos(2*L0*PI/180.0)
                     - 0.5*y*y*std::sin(4*L0*PI/180.0)
                     - 1.25*e*e*std::sin(2*M*PI/180.0) ) * 180.0/PI;

  double minutes = std::fmod((double)unix_utc/60.0, 1440.0);
  if(minutes<0) minutes+=1440.0;

  double lon_deg = lon_rad * 180.0/PI;
  double tst = minutes + Etime + 4.0*lon_deg;
  tst = std::fmod(tst, 1440.0);
  if(tst<0) tst+=1440.0;

  double ha_deg = (tst/4.0 < 0) ? (tst/4.0 + 180.0) : (tst/4.0 - 180.0);
  double ha = ha_deg * PI/180.0;

  double cosz = std::sin(lat_rad)*std::sin(decl) + std::cos(lat_rad)*std::cos(decl)*std::cos(ha);
  cosz = std::min(1.0,std::max(-1.0,cosz));
  double zen = std::acos(cosz);

  // robust azimuth from north clockwise
  double sinAz = -std::sin(ha)*std::cos(decl) / std::sin(zen);
  double cosAz = (std::sin(decl) - std::sin(lat_rad)*cosz) / (std::cos(lat_rad)*std::sin(zen));
  double azN = std::atan2(sinAz, cosAz);
  if(azN < 0) azN += 2*PI;
  return azN;
}

static double az_to_east_ccw(double az_north_cw){
  double alpha = (PI/2.0) - az_north_cw;
  while(alpha < 0) alpha += 2*PI;
  while(alpha >= 2*PI) alpha -= 2*PI;
  return alpha;
}

struct Stadium {
  double L=0, R=0;
  Vec2 C;
  double theta=0;
};

static bool stadium_inside_poly(const std::vector<Vec2>& poly, const Stadium& st, double eps_m){
  Vec2 u{std::cos(st.theta), std::sin(st.theta)};
  Vec2 A = st.C - u*(st.L/2.0);
  Vec2 B = st.C + u*(st.L/2.0);
  Seg AB{A,B};

  if(!point_in_poly(poly, A)) return false;
  if(!point_in_poly(poly, B)) return false;

  if(segment_intersects_poly_edges(poly, AB)){
    Vec2 mid = (A+B)*0.5;
    if(!point_in_poly(poly, mid)) return false;
  }

  double dmin = min_dist_seg_to_poly_edges(poly, AB);
  if(dmin + eps_m < st.R) return false;
  return true;
}

static Vec2 avg_point(const std::vector<Vec2>& pts){
  if(pts.empty()) return {0,0};
  Vec2 s{0,0};
  for(auto &p: pts) s = s + p;
  return s * (1.0 / pts.size());
}

static double normAngle(double a){
  while(a < 0) a += 2*PI;
  while(a >= 2*PI) a -= 2*PI;
  return a;
}

Result PlanRacetrack(const Inputs& in, const Config& cfg){
  Result res;

  // ---- Validate inputs ----
  if(in.safe_polygon.size() < 3){
    res.status = Status::InvalidInput;
    res.message = "safe_polygon must have at least 3 vertices.";
    return res;
  }
  if(in.L0 < 0 || in.R0 < 0){
    res.status = Status::InvalidInput;
    res.message = "L0/R0 must be non-negative.";
    return res;
  }
  if(cfg.earth_radius_m <= 0){
    res.status = Status::InvalidInput;
    res.message = "earth_radius_m must be positive.";
    return res;
  }

  const auto& poly = in.safe_polygon;

  // double w = maxx-minx, h = maxy-miny;
  // double cellSize = std::max(w,h);
  // if (!std::isfinite(cellSize) || cellSize < 1e-6) {
  //   res.status = Status::InvalidInput;
  //   res.message = "safe_polygon degenerate (bbox too small). Check coordinate transform.";
  //   return res;
  // }

  // ---- 1) Find open-area center (polylabel-like) ----
  Vec2 C0 = polylabel_center(poly, cfg.polylabel_precision_m);

  // ---- 2) Choose best approximate inscribed rectangle among 12 orientations ----
  Rect bestRect{C0, 0.0, 0.0, 0.0};
  for(int k=0;k<cfg.rect_orientation_count;k++){
    double th = deg2rad(cfg.rect_orientation_step_deg * k);
    Rect r = max_inscribed_rect_approx(poly, C0, th);
    if(r.area() > bestRect.area()) bestRect = r;
  }

  double rect_len = 2.0*bestRect.a;
  double rect_wid = 2.0*bestRect.b;

  // ---- 3) Initial track params (from empirical formula already in inputs) ----
  double L = in.L0;
  double R = in.R0;

  if(cfg.min_radius_m > 0) R = std::max(R, cfg.min_radius_m);

  // ---- 4) Quick shrink to fit rectangle ----
  if(cfg.shrink_to_fit_rect){
    R = std::min(R, rect_wid/2.0);
    L = std::min(L, rect_len - 2.0*R);
    if(L < 0) L = 0;
    if(R < 0) R = 0;
  }

  // ---- 5) Compute sun direction at rectangle center ----
  // Inverse transform: lat=y/Re, lon=x/Re (origin lat0=0 lon0=0)
  double lat = bestRect.c.y / cfg.earth_radius_m;
  double lon = bestRect.c.x / cfg.earth_radius_m;

  double azN = solar_azimuth_north_cw(lat, lon, in.unix_time_utc);
  double alpha_sun = az_to_east_ccw(azN);  // east=0 ccw

  double theta0 = normAngle(alpha_sun + PI/2.0); // perpendicular to sun

  // ---- 6) Orientation try sequence ----
  std::vector<double> thetaCandidates;
  thetaCandidates.reserve(cfg.orientation_offsets_deg.size() + 1);
  for(double offDeg : cfg.orientation_offsets_deg){
    thetaCandidates.push_back(normAngle(theta0 + deg2rad(offDeg)));
  }
  thetaCandidates.push_back(normAngle(bestRect.theta)); // fallback

  Vec2 ravg = avg_point(in.receivers);

  Vec2 ur{std::cos(bestRect.theta), std::sin(bestRect.theta)};
  Vec2 vr{-std::sin(bestRect.theta), std::cos(bestRect.theta)};

  auto rectLocalToWorld = [&](double xp, double yp){
    return bestRect.c + ur*xp + vr*yp;
  };

  // ---- 7) Try each orientation, generate ~70 candidate centers, collect feasible ----
  bool found = false;
  Stadium bestSt;
  double bestKey1 = std::numeric_limits<double>::infinity();
  double bestKey2 = std::numeric_limits<double>::infinity();

  for(double theta : thetaCandidates){
    double dtheta = theta - bestRect.theta;

    double px = std::fabs((L/2.0)*std::cos(dtheta)) + R;
    double py = std::fabs((L/2.0)*std::sin(dtheta)) + R;

    double xmax = bestRect.a - px;
    double ymax = bestRect.b - py;
    if(xmax <= 0 || ymax <= 0) continue;

    std::vector<std::pair<double,double>> cand;
    cand.reserve((cfg.add_structure_points? 12: 0) + cfg.halton_points);

    if(cfg.add_structure_points){
      // ~10 structure points
      cand.push_back({0,0});
      cand.push_back({ xmax,  ymax});
      cand.push_back({ xmax, -ymax});
      cand.push_back({-xmax, -ymax});
      cand.push_back({-xmax,  ymax});
      cand.push_back({ xmax,  0});
      cand.push_back({-xmax,  0});
      cand.push_back({ 0,  ymax});
      cand.push_back({ 0, -ymax});

      // bias point toward tanker
      Vec2 dt = in.tanker - bestRect.c;
      double txp = dot(dt, ur);
      double typ = dot(dt, vr);
      txp = std::max(-xmax, std::min(xmax, txp));
      typ = std::max(-ymax, std::min(ymax, typ));
      cand.push_back({txp, typ});
    }

    // Halton points
    for(int i=1;i<=cfg.halton_points;i++){
      double hx = halton(i,2);
      double hy = halton(i,3);
      double xp = (hx*2.0 - 1.0) * xmax;
      double yp = (hy*2.0 - 1.0) * ymax;
      cand.push_back({xp, yp});
    }

    std::vector<Stadium> feasible;
    feasible.reserve(cand.size());

    for(auto &pp : cand){
      Vec2 C = rectLocalToWorld(pp.first, pp.second);
      Stadium st{L, R, C, theta};
      if(stadium_inside_poly(poly, st, cfg.contain_eps_m)){
        feasible.push_back(st);
      }
    }

    if(feasible.empty()) continue;

    // select by model type and stop after first successful orientation (per your spec)
    for(const auto& st : feasible){
      double dT = norm(st.C - in.tanker);
      double dR = norm(st.C - ravg);

      double key1, key2;
      if(in.model_type == ModelType::PreferTanker){
        key1 = dT; key2 = dR;
      } else if(in.model_type == ModelType::PreferReceiver){
        key1 = dR; key2 = dT;
      } else { // Balanced0 / Balanced3
        key1 = std::fabs(dT - dR);
        key2 = dT + dR;
      }

      if(key1 < bestKey1 - 1e-12 || (std::fabs(key1-bestKey1)<1e-12 && key2 < bestKey2)){
        bestKey1 = key1;
        bestKey2 = key2;
        bestSt = st;
        found = true;
      }
    }
    if(found) break;
  }

  if(!found){
    res.status = Status::NoFeasibleSolution;
    res.message = "No feasible racetrack found under current L/R and constraints.";
    return res;
  }

  res.status = Status::Ok;
  res.out.L = bestSt.L;
  res.out.R = bestSt.R;
  res.out.center = bestSt.C;
  res.out.heading_deg = rad2deg(bestSt.theta);
  res.message = "ok";
  return res;
}

} // namespace racetrack
