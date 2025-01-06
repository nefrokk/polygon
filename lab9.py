import numpy as np
import unittest

# Функция, которая возвращает количество элементов в массиве, которые больше среднего арифметического
def count_greater_than_mean(array):
    
    # Возвращаем 0 для пустого массива
    if len(array) == 0:
        return 0 
    
    # Вычисляем среднее арифметическое
    mean_value = np.mean(array)
    
    # Считаем количество элементов, которые больше среднего
    count = np.sum(array > mean_value)
    
    # Возвращаем количество элементов, которые больше среднего
    return count

# Тесты для функции count_greater_than_mean
class TestCountGreaterThanMean(unittest.TestCase):
    
    # Тест для обычного случая
    def test_normal_case(self):
        array = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        self.assertEqual(count_greater_than_mean(array), 5)
        
    # Тест для случая пустого массива
    def test_empty_array(self):
        array = np.array([])
        self.assertEqual(count_greater_than_mean(array), 0)
        
    # Тест для случая массива из одинаковых элементов
    def test_all_elements_equal(self):
        array = np.array([5, 5, 5, 5])
        self.assertEqual(count_greater_than_mean(array), 0)
        
    # Тест для случая массива из одного элемента
    def test_single_element(self):
        array = np.array([10])
        self.assertEqual(count_greater_than_mean(array), 0)
        
    # Тест для случая массива с отрицательными элементами
    def test_negative_numbers(self):
        array = np.array([-10, -5, -1, 0, 1, 5, 10])
        self.assertEqual(count_greater_than_mean(array), 3)

if __name__ == "__main__":
    unittest.main()
