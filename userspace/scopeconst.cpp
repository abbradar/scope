#include <cstring>
#include "scopeconst.h"

const filter_row kFilterTable[] = {
  {SWEEP_WAVE, (uint32_char[]){
                 {50000, 7},
                 {150000, 6},
                 {300000, 5},
                 {500000, 4},
                 {700000, 2},
                 {1000000, 1},
                 {0, 0}
               }},
  {SINE_WAVE, (uint32_char[]){
                {50000, 7},
                {150000, 6},
                {300000, 5},
                {400000, 3},
                {500000, 2},
                {1000000, 1},
                {0, 0}
              }},
  {SINE_DIVX_WAVE, (uint32_char[]){
                     {5000, 7},
                     {50000, 6},
                     {500000, 1},
                     {0, 0}
                   }},
  {SQUARE_WAVE, (uint32_char[]){
                  {1000000, 0},
                  {0, 0}
                }},
  {DC_WAVE, (uint32_char[]){
              {1000000, 7},
              {0, 0}
            }},
  {OTHER_WAVE, (uint32_char[]){
                 {50000, 7},
                 {500000, 0},
                 {0, 0}
               }},
  {(filter_type)-1, NULL}
};
