#ifndef __MODEL_H
#define __MODEL_H

class Model
{
private:
  static char torcs_base[256];
private:
  int model;
  int child;
public:
  Model(const char* name);
  virtual ~Model();

  void  set(const int var, const float value);
  float compute();

  static void set_base(const char *base);
};

#endif
