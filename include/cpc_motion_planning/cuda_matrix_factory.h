#ifndef CUDA_MATRIX_FACTORY_H
#define CUDA_MATRIX_FACTORY_H
#include <cuda_math/cuda_matrix.cuh>
#include <vector>
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
  void* make_cuda_matrix(size_t *dim_width, T* data_ptr = nullptr)
  {
    Matrix<N,T> mat(dim_width);
    mat.setup_device();
    if (data_ptr != nullptr)
    {
      mat.upload_data(data_ptr);
    }

    void* ptr;
    CUDA_ALLOC_DEV_MEM(&ptr, sizeof(Matrix<N,T>));
    CUDA_MEMCPY_H2D(ptr, &mat, sizeof(Matrix<N,T>));

    return ptr;
  }

  template <int N, typename T>
  void free_cuda_matrix(void *ptr)
  {
    void* mat = malloc(sizeof(Matrix<N,T>));
    CUDA_MEMCPY_D2H(mat,ptr,sizeof(Matrix<N,T>));
    reinterpret_cast<Matrix<N,T>*>(mat)->free_device();
    CUDA_FREE_DEV_MEM(ptr);
    free(mat);
  }

  template <int N, typename T>
  void write_cuda_matrix(void *ptr, std::string file_name)
  {
    void* mat = malloc(sizeof(Matrix<N,T>));
    CUDA_MEMCPY_D2H(mat,ptr,sizeof(Matrix<N,T>));
    reinterpret_cast<Matrix<N,T>*>(mat)->write_to_file(file_name);
    free(mat);
  }

};
}
#endif // CUDA_MATRIX_FACTORY_H
