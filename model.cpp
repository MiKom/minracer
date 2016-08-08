#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "ffll/FFLLAPI.h"
#include "model.h"

char Model::torcs_base[256];

void Model::set_base(const char *base)
{
  sprintf(torcs_base, "%s/%s", getenv("HOME"), base);
}

Model::Model(const char *name)
{
  char path[256];
  sprintf(path, "%s/%s", torcs_base, name);

  model = ffll_new_model();
  if(ffll_load_fcl_file(model, path) < 0) {
    fprintf(stderr, "Model::load_fcl: %s\n", ffll_get_msg_text(model));
    exit(1);
  }
  child = ffll_new_child(model);
}

Model::~Model()
{
  ffll_close_model(model);
}

void Model::set(const int var, const float value)
{
  ffll_set_value(model, child, var, value);
}

float Model::compute()
{
  return ffll_get_output_value(model, child);
}
