#ifndef CUDA_MATRIX_FACTORY_H
#define CUDA_MATRIX_FACTORY_H
#include <cuda_math/cuda_matrix.cuh>
#include <vector>
#include <cpc_motion_planning/dynamic_programming.cuh>
namespace CUDA_MAT
{
class CudaMatrixFactory
{
public:
  CudaMatrixFactory()
  {

  }

  ~CudaMatrixFactory()
  {

  }

  template <int N, typename T>
  void* make_cuda_matrix(size_t *dim_width, T* data_ptr = NULL)
  {
    Matrix<N,T> mat(dim_width);
    mat.setup_device();
    if (data_ptr != NULL)
    {
      mat.upload_data(data_ptr);
    }

    void* ptr;
    CUDA_ALLOC_DEV_MEM(&ptr, sizeof(Matrix<N,T>));
    CUDA_MEMCPY_H2D(ptr, &mat, sizeof(Matrix<N,T>));

    return ptr;
  }

  template <int N,typename T>
  void* load_cuda_matrix(std::string file_name, bool load_to_host=false)
  {
    // open the file
    std::ifstream file;
    file.open(file_name.c_str(), std::ios::binary);

    if (!file.is_open())
    {
      std::cout<<"File mat open failed."<<std::endl;
      file.close();
      return NULL;
    }

    int Dim;
    file.read(reinterpret_cast<char *>(&Dim),sizeof(int));

    if (Dim <= 0 || Dim != N)
    {
      std::cout<<"Matrix dimension wrong."<<std::endl;
      file.close();
      return NULL;
    }

    // read the header to determine the size of the matrix
    int byte_size = 1;
    int size=1;
    size_t dim_width[N];
    for (int i=0; i<Dim; i++)
    {
      int width;
      file.read(reinterpret_cast<char *>(&width),sizeof(int));
      dim_width[i] = width;
      byte_size *= dim_width[i];
      size *= dim_width[i];
    }
    byte_size *= sizeof(T);

    // make matrix
    Matrix<N,T> mat(dim_width,load_to_host);

    // create the device buffers
    mat.setup_device();

    // load the data onto the device
    T *data = new T[size];
    file.read(reinterpret_cast<char *>(data),byte_size);
    mat.upload_data(data);
    delete [] data;

    // close the file
    file.close();

    void* ptr;
    if(!load_to_host)
    {
      CUDA_ALLOC_DEV_MEM(&ptr, sizeof(Matrix<N,T>));
      CUDA_MEMCPY_H2D(ptr, &mat, sizeof(Matrix<N,T>));
    }
    else
    {
      ptr = malloc(sizeof(Matrix<N,T>));
      memcpy(ptr,&mat,sizeof(Matrix<N,T>));
    }

    return ptr;
  }

  void load_uniform_bin(std::string file_name,  UniformBin &bin)
  {
    // open the file
    std::ifstream file;
    file.open(file_name.c_str(), std::ios::binary);

    if (!file.is_open())
    {
      std::cout<<"File bin open failed."<<std::endl;
      file.close();
      return;
    }

    int Dim;
    file.read(reinterpret_cast<char *>(&Dim),sizeof(int));

    if (Dim != 1)
    {
      std::cout<<"Matrix dimension is not 1. Not a bin file."<<std::endl;
      file.close();
      return ;
    }

    // read the header to determine the size of the matrix
    int width;
    file.read(reinterpret_cast<char *>(&width),sizeof(int));

    int byte_size = width * static_cast<int>(sizeof(float));

    // load the data onto the device
    float *data = new float[width];
    file.read(reinterpret_cast<char *>(data),byte_size);

    bin.min = data[0];
    bin.grid = data[1] - data[0];
    bin.size = width;
    bin.max = bin.min + (bin.size - 1)*bin.grid;

    delete [] data;
    return;
  }


  template <int N, typename T>
  void free_cuda_matrix(void *ptr, bool load_from_host=false)
  {
    if (!load_from_host)
    {
      void* mat = malloc(sizeof(Matrix<N,T>));
      CUDA_MEMCPY_D2H(mat,ptr,sizeof(Matrix<N,T>));
      reinterpret_cast<Matrix<N,T>*>(mat)->free_device();
      CUDA_FREE_DEV_MEM(ptr);
      free(mat);
    }
    else
    {
      Matrix<N,T>* mat = static_cast<Matrix<N,T>*>(ptr);
      mat->free_device();
      free(mat);
    }
  }

  template <int N, typename T>
  void write_cuda_matrix(void *ptr, std::string file_name, bool load_from_host=false)
  {
    if (!load_from_host)
    {
      void* mat = malloc(sizeof(Matrix<N,T>));
      CUDA_MEMCPY_D2H(mat,ptr,sizeof(Matrix<N,T>));
      reinterpret_cast<Matrix<N,T>*>(mat)->write_to_file(file_name);
      free(mat);
    }
    else
    {
      Matrix<N,T>* mat = static_cast<Matrix<N,T>*>(ptr);
      mat->write_to_file(file_name);
    }
  }

};
}
#endif // CUDA_MATRIX_FACTORY_H
