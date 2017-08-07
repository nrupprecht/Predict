// Template class definition file - to be included in FieldBase.hpp

template<typename T> FieldBase<T>::FieldBase() : dx(0), dy(0), idx(0), idy(0), nx(50), ny(50), wrap(true) {};

template<typename T> FieldBase<T>::FieldBase(const FieldBase& f) : dx(f.dx), dy(f.dy), idx(f.idx), idy(f.idy), nx(f.nx), ny(f.ny), wrap(f.wrap) {
  bounds = f.bounds;
  // Resize the data array
  if (nx*ny>0)
    data.resize(nx*ny);
  else data.reset();
  // Copy values
  for (int y=0; y<ny; ++y)
    for (int x=0; x<nx; ++x)
      at(x,y) = f.at(x,y);
}

template<typename T> FieldBase<T>::FieldBase(const Bounds& b) : bounds(b), nx(50), ny(50), wrap(true) {
  initialize();
};

template<typename T> FieldBase<T>::FieldBase(const Bounds& b, int n) : bounds(b), nx(n), ny(n), wrap(true) {
  initialize();
};

template<typename T> FieldBase<T>::~FieldBase() {};

template<typename T> void FieldBase<T>::initialize() {
  // Assumes bounds, nx, and ny are correct, other variables need reset
  dx = (bounds.right - bounds.left)/nx;
  dy = (bounds.top - bounds.bottom)/ny;
  idx = 1./dx; idy = 1./dy;
  // Resize the data array
  if (nx*ny>0)
    data.resize(nx*ny);
  else data.reset();
}

template<typename T> void FieldBase<T>::set(std::function<RealType(vec2)> func) {
  for (int y=0; y<ny; ++y)
    for (int x=0; x<nx; ++x) {
      vec2 p = getPosition(x,y);
      data.at(y*nx+x) = func(p);
    }
}

template<typename T> void FieldBase<T>::set(T value) {
  for (int y=0; y<ny; ++y)
    for (int x=0; x<nx; ++x)
      data.at(y*nx+x) = value;
}

template<typename T> FieldBase<T>& FieldBase<T>::operator=(const FieldBase<T>& field) {
  dx = field.dx;   dy = field.dy;
  idx = field.idx; idy = field.idy;
  nx = field.nx;   ny = field.ny;
  bounds = field.bounds;
  wrap = field.wrap;
  if (field.data.empty()) data.reset();
  else {
    data.resize(field.data.size());
    for (int i=0; i<field.data.size(); ++i) 
      data.at(i) = field.data.at(i);
  }
}

template<typename T> T& FieldBase<T>::at(int x, int y) {
  if (wrap) {
    if (nx<=x)    x %= nx;
    else if (x<0) x = nx - (-x) % nx;
    if (ny<=y)    y %= ny;
    else if (y<0) y = ny - (-y) % ny;
  }
  return data.at(nx*y + x);
}

template<typename T> T FieldBase<T>::at(int x, int y) const {
  if (wrap) {
    if (nx<=x)    x %= nx;
    else if (x<0) x = nx - (-x) % nx;
    if (ny<=y)    y %= ny;
    else if (y<0) y = ny - (-y) % ny;
  }
  return data.at(nx*y + x);
}

template<typename T> int FieldBase<T>::getBin(vec2 pos) const {
  double X = (pos.x-bounds.left)*idx, Y = (pos.y-bounds.bottom)*idy;
  return nx*static_cast<int>(Y) + static_cast<int>(X);
}

template<typename T> void FieldBase<T>::setNX(int n) {
  nx = n;
  initialize();
}

template<typename T> void FieldBase<T>::setNY(int n) {
  ny = n;
  initialize();
}

template<typename T> void FieldBase<T>::setN(int n) {
  nx = ny = n;
  initialize();
}

template<typename T> void FieldBase<T>::setWrap(bool w) {
  wrap = w;
}

template<typename T> vec2 FieldBase<T>::getPosition(int x, int y) const {
  // Data is stored for the center of the points, so the 0th point corresponds
  //  to 0.5*dx, etc.
  return vec2( (0.5+x)*dx+bounds.left, (0.5+y)*dy+bounds.bottom );
}

template<typename T> T FieldBase<T>::get(vec2 pos) const {
  // Get Border points
  //  tl *   * tr
  //       x
  //  bl *   * br
  // -------------
  double X = (pos.x-bounds.left)*idx, Y = (pos.y-bounds.bottom)*idy;
  double bx = (int)X, by = (int)Y;
  double x = X-bx, y = Y-by; // in [0,1]
  T tl = at(bx, by+1), tr = at(bx+1, by+1);
  T bl = at(bx, by), br = at(bx+1, by);
  T p_E = x*(br-bl) + bl; // Bottom
  T p_F= x*(tr-tl) + tl;  // Top
  return y*(p_F-p_E) + p_E;
}

template<typename T> T& FieldBase<T>::get(vec2 pos) {
  int x = static_cast<int>((pos.x-bounds.left)*idx), y = static_cast<int>((pos.y-bounds.bottom)*idy);
  return data.at(nx*y + x);
}

template<typename T> T FieldBase<T>::DX(vec2 pos) const {
  // Approximate derivative as derivative at left/bottom grid point
  double X = (pos.x-bounds.left)*idx, Y = (pos.y-bounds.bottom)*idy;
  double x = (int)X, y = (int)Y;
  if (wrap || (x!=0 && x!=nx-1))
    return 0.5*idx*(at(x+1,y) - at(x-1,y));
  else if (x==0)    return idx*(at(x+1,y)-at(x,y));
  else if (x==nx-1) return idx*(at(x,y)-at(x-1,y));
}

template<typename T> T FieldBase<T>::DY(vec2 pos) const {
  // Approximate derivative as derivative at closest grid point
  double X = (pos.x-bounds.left)*idx, Y = (pos.y-bounds.bottom)*idy;
  double x = (int)X, y = (int)Y;
  if (wrap || (y!=0 && y!=ny-1))
    return 0.5*idy*(at(x,y+1) - at(x,y-1));
  else if (y==0)    return idy*(at(x,y+1)-at(x,y));
  else if (y==ny-1) return idy*(at(x,y)-at(x,y-1));
}

template<typename T> T FieldBase<T>::DX(int x, int y) const {
  if (wrap || (x!=0 && x!=nx-1))
    return 0.5*idx*(at(x+1,y) - at(x-1,y));
  else if (x==0)    return idx*(at(1,y)-at(0,y));
  else if (x==nx-1) return idx*(at(nx-1,y)-at(nx-2,y));
}

template<typename T> T FieldBase<T>::DY(int x, int y) const {
  if (wrap || (y!=0 && y!=ny-1))
    return 0.5*idy*(at(x,y+1) - at(x,y-1));
  else if (y==0)    return idy*(at(x,1)-at(x,0));
  else if (y==ny-1) return idy*(at(x,ny-1)-at(x,ny-2));
}

template<typename T> T FieldBase<T>::DX2(vec2 pos) const {
  // Approximate derivative as derivative at left/bottom grid point
  double X = (pos.x-bounds.left)*idx, Y = (pos.y-bounds.bottom)*idy;
  double x = (int)X, y = (int)Y;
  if (wrap || (x!=0 && x!=nx-1))
    return sqr(idx)*(at(x+1,y) - 2*at(x,y) + at(x-1,y));
  else if (x==0)    return sqr(idx)*(at(2,y)-2*at(1,y)+at(0,y));
  else if (x==nx-1) return sqr(idx)*(at(nx-1,y)-2*at(nx-2,y)+at(nx-3,y));
}

template<typename T> T FieldBase<T>::DY2(vec2 pos) const {
  // Approximate derivative as derivative at closest grid point
  double X = (pos.x-bounds.left)*idx, Y = (pos.y-bounds.bottom)*idy;
  double x = (int)X, y = (int)Y;
  if (wrap || (y!=0 && y!=ny-1))
    return sqr(idy)*(at(x,y+1) - 2*at(x,y) + at(x,y-1));
  else if (y==0)    return sqr(idy)*(at(x,0)-2*at(x,1)+at(x,2));
  else if (y==ny-1) return sqr(idy)*(at(x,ny-1)-2*at(x,ny-2)+at(x,ny-3));
}

template<typename T> T FieldBase<T>::DX2(int x, int y) const {
  if (wrap || (x!=0 && x!=nx-1))
    return sqr(idx)*(at(x+1,y) - 2*at(x,y) + at(x-1,y));
  else if (x==0)    return sqr(idx)*(at(2,y)-2*at(1,y)+at(0,y));
  else if (x==nx-1) return sqr(idx)*(at(nx-1,y)-2*at(nx-2,y)+at(nx-3,y));
}

template<typename T> T FieldBase<T>::DY2(int x, int y) const {
  if (wrap || (y!=0 && y!=ny-1))
    return sqr(idy)*(at(x,y+1) - 2*at(x,y) + at(x,y-1));
  else if (y==0)    return sqr(idy)*(at(x,0)-2*at(x,1)+at(x,2));
  else if (y==ny-1) return sqr(idy)*(at(x,ny-1)-2*at(x,ny-2)+at(x,ny-3));
}

template<typename T> T FieldBase<T>::Lap(int x, int y) const {
  if (wrap || (x!=0 && x!=nx-1 && y!=0 && y!=ny-1)) {
    if (dx==dy)
      return sqr(idx)*(at(x+1,y) + at(x-1,y) + at(x,y+1) + at(x,y-1) - 4*at(x,y));
    else return sqr(idx)*(at(x-1,y) + at(x+1,y) - 2*at(x,y)) + sqr(idy)*(at(x,y-1) + at(x,y+1) - 2*at(x,y));
  }
  // Normal way of taking the laplacian
  else return DX2(x,y) + DY2(x,y);
}

template<typename T> void FieldBase<T>::minusEq(FieldBase<T> &field, RealType mult, bool limit) {
  if (nx!=field.nx || ny!=field.ny) throw UnalignedPoints();
  if (dx!=field.dx || dy!=field.dy) throw UnalignedSpacing();
  // Do subtraction
  for (int y=0; y<ny; ++y)
    for (int x=0; x<nx; ++x) {
      at(x,y) -= mult*field.at(x,y);
      if (limit && at(x,y)<0) at(x,y) = 0;
    }
}

template<typename T> void FieldBase<T>::plusEq(FieldBase<T> &field, RealType mult, bool limit) {
  if (nx!=field.nx || ny!=field.ny) throw UnalignedPoints();
  if (dx!=field.dx || dy!=field.dy) throw UnalignedSpacing();
  if (bounds!=field.bounds) throw UnalignedBounds();
  // Do subtraction
  for (int y=0; y<ny; ++y)
    for (int x=0; x<nx; ++x) {
      at(x,y) += mult*field.at(x,y);
      if (limit && at(x,y)<0) at(x,y) = 0;
    }
}

template<typename T> T FieldBase<T>::total() const {
  T total = T();
  // Total up the amount of stuff in the field
  for (int y=0; y<ny; ++y)
    for (int x=0; x<nx; ++x)
      total += at(x,y);
  return bounds.volume()/static_cast<RealType>(nx*ny) * total;
}
