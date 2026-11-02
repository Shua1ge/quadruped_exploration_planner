#include "bspline_opt/uniform_bspline.h"

namespace scan_planner
{

  UniformBspline::UniformBspline(const Eigen::MatrixXd &points, const int &order,
                                 const double &interval)
  {
    setUniformBspline(points, order, interval);
  }

  UniformBspline::~UniformBspline() {}

  void UniformBspline::setUniformBspline(const Eigen::MatrixXd &points, const int &order,
                                         const double &interval)
  {
    control_points_ = points;
    p_ = order;
    interval_ = interval;

    n_ = points.cols() - 1;
    m_ = n_ + p_ + 1;

    u_ = Eigen::VectorXd::Zero(m_ + 1);
    for (int i = 0; i <= m_; ++i)
    {

      if (i <= p_)
      {
        u_(i) = double(-p_ + i) * interval_;
      }
      else if (i > p_ && i <= m_ - p_)
      {
        u_(i) = u_(i - 1) + interval_;
      }
      else if (i > m_ - p_)
      {
        u_(i) = u_(i - 1) + interval_;
      }
    }
  }

  void UniformBspline::setKnot(const Eigen::VectorXd &knot) { this->u_ = knot; }

  Eigen::VectorXd UniformBspline::getKnot() { return this->u_; }

  bool UniformBspline::getTimeSpan(double &um, double &um_p)
  {
    if (p_ > u_.rows() || m_ - p_ > u_.rows())
      return false;

    um = u_(p_);
    um_p = u_(m_ - p_);

    return true;
  }

  Eigen::MatrixXd UniformBspline::getControlPoint() { return control_points_; }

  Eigen::VectorXd UniformBspline::evaluateDeBoor(const double &u) const
  {

    double ub = min(max(u_(p_), u), u_(m_ - p_));

    // determine which [ui,ui+1] lay in
    int k = p_;
    while (true)
    {
      if (u_(k + 1) >= ub)
        break;
      ++k;
    }

    /* deBoor's alg */
    vector<Eigen::VectorXd> d;
    for (int i = 0; i <= p_; ++i)
    {
      d.push_back(control_points_.col(k - p_ + i));
      // cout << d[i].transpose() << endl;
    }

    for (int r = 1; r <= p_; ++r)
    {
      for (int i = p_; i >= r; --i)
      {
        double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
        // cout << "alpha: " << alpha << endl;
        d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
      }
    }

    return d[p_];
  }

  // Eigen::VectorXd UniformBspline::evaluateDeBoorT(const double& t) {
  //   return evaluateDeBoor(t + u_(p_));
  // }

  Eigen::MatrixXd UniformBspline::getDerivativeControlPoints()
  {
    // The derivative of a b-spline is also a b-spline, its order become p_-1
    // control point Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
    Eigen::MatrixXd ctp(control_points_.rows(), control_points_.cols() - 1);
    for (int i = 0; i < ctp.cols(); ++i)
    {
      ctp.col(i) =
          p_ * (control_points_.col(i + 1) - control_points_.col(i)) / (u_(i + p_ + 1) - u_(i + 1));
    }
    return ctp;
  }

  UniformBspline UniformBspline::getDerivative()
  {
    Eigen::MatrixXd ctp = getDerivativeControlPoints();
    UniformBspline derivative(ctp, p_ - 1, interval_);

    /* cut the first and last knot */
    Eigen::VectorXd knot(u_.rows() - 2);
    knot = u_.segment(1, u_.rows() - 2);
    derivative.setKnot(knot);

    return derivative;
  }

  double UniformBspline::getInterval() { return interval_; }

  void UniformBspline::setPhysicalLimits(const double &vel, const double &acc, const double &tolerance)
  {
    limit_vel_ = vel;
    limit_acc_ = acc;
    feasibility_tolerance_ = tolerance;
  }

  bool UniformBspline::checkFeasibility(double &ratio, bool show)
  {
    bool fea = true;

    Eigen::MatrixXd P = control_points_;
    int dimension = control_points_.rows();

    /* check vel feasibility and insert points */
    double max_vel = -1.0;
    double enlarged_vel_lim = limit_vel_ * (1.0 + feasibility_tolerance_) + 1e-4;
    for (int i = 0; i < P.cols() - 1; ++i)
    {
      Eigen::VectorXd vel = p_ * (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1));

      if (fabs(vel(0)) > enlarged_vel_lim || fabs(vel(1)) > enlarged_vel_lim ||
          fabs(vel(2)) > enlarged_vel_lim)
      {

        if (show)
          cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << endl;
        fea = false;

        for (int j = 0; j < dimension; ++j)
        {
          max_vel = max(max_vel, fabs(vel(j)));
        }
      }
    }

    /* acc feasibility */
    double max_acc = -1.0;
    double enlarged_acc_lim = limit_acc_ * (1.0 + feasibility_tolerance_) + 1e-4;
    for (int i = 0; i < P.cols() - 2; ++i)
    {

      Eigen::VectorXd acc = p_ * (p_ - 1) *
                            ((P.col(i + 2) - P.col(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
                             (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
                            (u_(i + p_ + 1) - u_(i + 2));

      if (fabs(acc(0)) > enlarged_acc_lim || fabs(acc(1)) > enlarged_acc_lim ||
          fabs(acc(2)) > enlarged_acc_lim)
      {

        if (show)
          cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << endl;
        fea = false;

        for (int j = 0; j < dimension; ++j)
        {
          max_acc = max(max_acc, fabs(acc(j)));
        }
      }
    }

    ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));

    return fea;
  }

  void UniformBspline::lengthenTime(const double &ratio)
  {
    int num1 = 5;
    int num2 = getKnot().rows() - 1 - 5;

    double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));
    double t_inc = delta_t / double(num2 - num1);
    for (int i = num1 + 1; i <= num2; ++i)
      u_(i) += double(i - num1) * t_inc;
    for (int i = num2 + 1; i < u_.rows(); ++i)
      u_(i) += delta_t;
  }

  // void UniformBspline::recomputeInit() {}

  void UniformBspline::parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                             const vector<Eigen::Vector3d> &start_end_derivative,
                                             Eigen::MatrixXd &ctrl_pts)
  {
    if (ts <= 0)
    {
      cout << "[B-spline]:time step error." << endl;
      return;
    }

    if (point_set.size() <= 3)
    {
      cout << "[B-spline]:point set have only " << point_set.size() << " points." << endl;
      return;
    }

    if (start_end_derivative.size() != 4)
    {
      cout << "[B-spline]:derivatives error." << endl;
    }

    int K = point_set.size();

    // write A
    Eigen::Vector3d prow(3), vrow(3), arow(3);
    prow << 1, 4, 1;
    vrow << -1, 0, 1;
    arow << 1, -2, 1;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);

    for (int i = 0; i < K; ++i)
      A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();

    A.block(K, 0, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();
    A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();

    A.block(K + 2, 0, 1, 3) = (1 / ts / ts) * arow.transpose();
    A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();

    //cout << "A" << endl << A << endl << endl;

    // write b
    Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);
    for (int i = 0; i < K; ++i)
    {
      bx(i) = point_set[i](0);
      by(i) = point_set[i](1);
      bz(i) = point_set[i](2);
    }

    for (int i = 0; i < 4; ++i)
    {
      bx(K + i) = start_end_derivative[i](0);
      by(K + i) = start_end_derivative[i](1);
      bz(K + i) = start_end_derivative[i](2);
    }

    // solve Ax = b
    Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
    Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

    // convert to control pts
    ctrl_pts.resize(3, K + 2);
    ctrl_pts.row(0) = px.transpose();
    ctrl_pts.row(1) = py.transpose();
    ctrl_pts.row(2) = pz.transpose();

    // cout << "[B-spline]: parameterization ok." << endl;
  }

  double UniformBspline::getTimeSum()
  {
    double tm, tmp;
    if (getTimeSpan(tm, tmp))
      return tmp - tm;
    else
      return -1.0;
  }

  double UniformBspline::getLength(const double &res)
  {
    double length = 0.0;
    double dur = getTimeSum();
    Eigen::VectorXd p_l = evaluateDeBoorT(0.0), p_n;
    for (double t = res; t <= dur + 1e-4; t += res)
    {
      p_n = evaluateDeBoorT(t);
      length += (p_n - p_l).norm();
      p_l = p_n;
    }
    return length;
  }

  double UniformBspline::getJerk()
  {
    UniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

    Eigen::VectorXd times = jerk_traj.getKnot();
    Eigen::MatrixXd ctrl_pts = jerk_traj.getControlPoint();
    int dimension = ctrl_pts.rows();

    double jerk = 0.0;
    for (int i = 0; i < ctrl_pts.cols(); ++i)
    {
      for (int j = 0; j < dimension; ++j)
      {
        jerk += (times(i + 1) - times(i)) * ctrl_pts(j, i) * ctrl_pts(j, i);
      }
    }

    return jerk;
  }

  void UniformBspline::getMeanAndMaxVel(double &mean_v, double &max_v)
  {
    UniformBspline vel = getDerivative();
    double tm, tmp;
    vel.getTimeSpan(tm, tmp);

    double max_vel = -1.0, mean_vel = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
      double vn = vxd.norm();

      mean_vel += vn;
      ++num;
      if (vn > max_vel)
      {
        max_vel = vn;
      }
    }

    mean_vel = mean_vel / double(num);
    mean_v = mean_vel;
    max_v = max_vel;
  }

  void UniformBspline::getMeanAndMaxAcc(double &mean_a, double &max_a)
  {
    UniformBspline acc = getDerivative().getDerivative();
    double tm, tmp;
    acc.getTimeSpan(tm, tmp);

    double max_acc = -1.0, mean_acc = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::VectorXd axd = acc.evaluateDeBoor(t);
      double an = axd.norm();

      mean_acc += an;
      ++num;
      if (an > max_acc)
      {
        max_acc = an;
      }
    }

    mean_acc = mean_acc / double(num);
    mean_a = mean_acc;
    max_a = max_acc;
  }

  namespace {
  bool insertExactKnot(Eigen::MatrixXd& points,std::vector<double>& knots,int degree,double t) {
    const int n=points.cols()-1;
    const int multiplicity=std::count(knots.begin(),knots.end(),t);
    if(multiplicity>=degree+1) return true;
    const int k=std::upper_bound(knots.begin(),knots.end(),t)-knots.begin()-1;
    if(k<degree || k>n+1) return false;
    Eigen::MatrixXd next(points.rows(),n+2);
    for(int i=0;i<=k-degree;++i) next.col(i)=points.col(i);
    for(int i=k-multiplicity;i<=n;++i) next.col(i+1)=points.col(i);
    for(int i=k-degree+1;i<=k-multiplicity;++i) {
      const double denominator=knots[i+degree]-knots[i];
      if(denominator<=0 || i<=0 || i>n) return false;
      const double a=(t-knots[i])/denominator;
      next.col(i)=a*points.col(i)+(1-a)*points.col(i-1);
    }
    knots.insert(knots.begin()+k+1,t);points=std::move(next);
    return points.allFinite();
  }
  // Invert one Boehm insertion. C2-compatible cubic joins allow the two
  // redundant interior knots to be removed without changing either segment.
  bool removeExactKnot(Eigen::MatrixXd& q,std::vector<double>& knots,int degree,double t) {
    const auto at=std::upper_bound(knots.begin(),knots.end(),t);
    if(at==knots.begin() || *(at-1)!=t) return false;
    auto u=knots;u.erase(u.begin()+(at-knots.begin()-1));
    const int k=std::upper_bound(u.begin(),u.end(),t)-u.begin()-1;
    const int count=std::count(u.begin(),u.end(),t),n=q.cols()-2;
    const int right=k-count;
    if(k-degree<0 || right<0 || right>n) return false;
    Eigen::MatrixXd p(q.rows(),n+1);
    for(int i=0;i<=k-degree;++i) p.col(i)=q.col(i);
    for(int i=right;i<=n;++i) p.col(i)=q.col(i+1);
    for(int i=k-degree+1;i<=right;++i) {
      const double den=u[i+degree]-u[i],a=den>0 ? (t-u[i])/den : 0.;
      if(a<=1e-12) return false;
      const Eigen::VectorXd recovered=(q.col(i)-(1-a)*p.col(i-1))/a;
      if(i==right) {if((p.col(i)-recovered).norm()>1e-6) return false;}
      else p.col(i)=recovered;
    }
    q=std::move(p);knots=std::move(u);return q.allFinite();
  }
  }

  std::optional<UniformBspline> UniformBspline::segment(double begin,double end) const {
    if(p_<1 || control_points_.cols()<=p_ || !control_points_.allFinite() ||
       !u_.allFinite() || !std::isfinite(begin) || !std::isfinite(end) ||
       begin<0 || end<=begin+1e-8 || end>u_(m_-p_)-u_(p_)+1e-9) return {};
    const double a=begin+u_(p_),z=std::min(end+u_(p_),u_(m_-p_));
    auto points=control_points_;std::vector<double> knots(u_.data(),u_.data()+u_.size());
    for(double t:{a,z}) while(std::count(knots.begin(),knots.end(),t)<p_+1)
      if(!insertExactKnot(points,knots,p_,t)) return {};
    const int first=std::lower_bound(knots.begin(),knots.end(),a)-knots.begin();
    const int last=std::upper_bound(knots.begin(),knots.end(),z)-knots.begin();
    const int count=last-first-p_-1;
    if(count<=p_ || first+count>points.cols()) return {};
    UniformBspline result(points.middleCols(first,count),p_,interval_);
    Eigen::VectorXd u(last-first);
    for(int i=0;i<u.size();++i) u(i)=knots[first+i]-a;
    result.setKnot(u);return result;
  }

  std::optional<UniformBspline> UniformBspline::joinC2(
      const UniformBspline& prefix,const UniformBspline& suffix) {
    if(prefix.p_!=3 || suffix.p_!=3 || prefix.control_points_.rows()!=suffix.control_points_.rows()) return {};
    auto left=prefix.segment(0.,prefix.u_(prefix.m_-prefix.p_)-prefix.u_(prefix.p_));
    auto right=suffix.segment(0.,suffix.u_(suffix.m_-suffix.p_)-suffix.u_(suffix.p_));
    if(!left || !right) return {};
    const double duration=left->getTimeSum();
    auto lv=left->getDerivative(),rv=right->getDerivative();
    auto la=lv.getDerivative(),ra=rv.getDerivative();
    if((left->evaluateDeBoorT(duration)-right->evaluateDeBoorT(0)).norm()>1e-7 ||
       (lv.evaluateDeBoorT(duration)-rv.evaluateDeBoorT(0)).norm()>1e-6 ||
       (la.evaluateDeBoorT(duration)-ra.evaluateDeBoorT(0)).norm()>1e-5) return {};
    const auto& a=left->control_points_;const auto& b=right->control_points_;
    Eigen::MatrixXd points(a.rows(),a.cols()+b.cols()-1);
    points.leftCols(a.cols())=a;points.rightCols(b.cols()-1)=b.rightCols(b.cols()-1);
    std::vector<double> knots;
    for(int i=0;i<left->u_.size()-1;++i) knots.push_back(left->u_(i));
    for(int i=4;i<right->u_.size();++i) knots.push_back(duration+right->u_(i));
    if(!removeExactKnot(points,knots,3,duration) || !removeExactKnot(points,knots,3,duration)) return {};
    UniformBspline result(points,3,prefix.interval_);
    result.setKnot(Eigen::Map<Eigen::VectorXd>(knots.data(),knots.size()));
    auto v=result.getDerivative();auto acc=v.getDerivative();
    if(!v.getControlPoint().allFinite() || !acc.getControlPoint().allFinite()) return {};
    return result;
  }
} // namespace scan_planner
