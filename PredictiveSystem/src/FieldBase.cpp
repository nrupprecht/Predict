// Template class definition file - to be included in FieldBase.hpp

template<typename T> FieldBase<T>::FieldBase() : dx(0), dy(0), idx(0), idy(0), nx(50), ny(50), wrap(true) {};

template<typename T> FieldBase<T>::FieldBase(const Bounds b) : bounds(b), nx(50), ny(50), wrap(true) {
  initialize();
};

template<typename T> FieldBase<T>::~FieldBase() {};

template<typename T> void FieldBase<T>::initialize() {
  // Assumes bounds, nx, and ny are correct, other variables need reset
  if (wrap) {
    dx = (bounds.right - bounds.left)/nx;
    dy = (bounds.top - bounds.bottom)/ny;
  }
  else {
    dx = (bounds.right - bounds.left)/(nx-1);
    dy = (bounds.top - bounds.bottom)/(ny-1);
  }
  idx = 1./dx; idy = 1./dy;
  data.resize(nx*ny);
}

template<typename T> T& FieldBase<T>::at(int x, int y) {
  return data.at(nx*y + x);
}

template<typename T> T FieldBase<T>::get(vec2 pos) {
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

template<typename T> T FieldBase<T>::DX(vec2 pos) {
  // Approximate derivative as derivative at left/bottom grid point
  double X = (pos.x-bounds.left)*idx, Y = (pos.y-bounds.bottom)*idy;
  double x = (int)X, y = (int)Y;
  if (wrap || (x!=0 || x!=nx-1))
    return sqr(idx)*(at(x+1) - 2*at(x,y) + at(x-1));
  else if (x==0)    return idx*(at(x+1,y)-at(x,y));
  else if (x==nx-1) return idx*(at(x,y)-at(x-1,y));
}

template<typename T> T FieldBase<T>::DY(vec2 pos) {
  // Approximate derivative as derivative at closest grid point
  double X = (pos.x-bounds.left)*idx, Y = (pos.y-bounds.bottom)*idy;
  double x = (int)X, y = (int)Y;
  if (wrap || (y!=0 || y!=ny-1))
    return sqr(idy)*(at(x,y+1) - 2*at(x,y) + at(x,y-1));
  else if (y==0)    return idy*(at(x,y+1)-at(x,y));
  else if (y==ny-1) return idy*(at(x,y)-at(x,y-1));
}
