
#include <signal.h>
#include <getopt.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cassert>
#include <string>

#define MAX_CLASSES 1000
#define MAX_SELECTED_ITEMS 100

using namespace std;

std::string labels_classes[MAX_CLASSES];
int IMAGE_CLASSES_NUM = 0;
int selected_items_size = 0;
int selected_items[MAX_SELECTED_ITEMS];
static int get_classindex(std::string str2find)
{
  if(selected_items_size >= MAX_SELECTED_ITEMS)
  {
     std::cout << "Max number of selected classes is reached! (" << selected_items_size << ")!" << std::endl;
     return -1;
  }
  for (int i = 0; i < IMAGE_CLASSES_NUM; i ++)
  {
    if(labels_classes[i].compare(str2find) == 0)
    {
      selected_items[selected_items_size ++] = i;
      return i;
    }
  }
  std::cout << "Not found: " << str2find << std::endl << std::flush;
  return -1;
}

int populate_selected_items (char *filename)
{
  ifstream file(filename);
  if(file.is_open())
  {
    string inputLine;

    while (getline(file, inputLine) )                 //while the end of file is NOT reached
    {
      int res = get_classindex(inputLine);
      std::cout << "Searching for " << inputLine  << std::endl;
      if(res >= 0) {
        std::cout << "Found: " << res << std::endl;
      } else {
        std::cout << "Not Found: " << res << std::endl;
      }
    }
    file.close();
  }
#if 0
  std::cout << "==Total of " << selected_items_size << " items!" << std::endl;
  for (int i = 0; i < selected_items_size; i ++)
    std::cout << i << ") " << selected_items[i] << std::endl;
#endif
  return selected_items_size;
}

void populate_labels (char *filename)
{
  ifstream file(filename);
  if(file.is_open())
  {
    string inputLine;

    while (getline(file, inputLine) )                 //while the end of file is NOT reached
    {
      labels_classes[IMAGE_CLASSES_NUM ++] = string(inputLine);
    }
    file.close();
  }
#if 1
  std::cout << "==Total of " << IMAGE_CLASSES_NUM << " items!" << std::endl;
  for (int i = 0; i < IMAGE_CLASSES_NUM; i ++)
    std::cout << i << ") " << labels_classes[i] << std::endl;
#endif
}


