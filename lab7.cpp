#include <iostream>
#include <vector>

using namespace std;

int candy(vector<int>& ratings) {
    int n = ratings.size(); // O(1)
    int totalCandies = n; // O(1)
    int i = 1; // O(1)
    
    //Проходит по всем элементам вектора ratings
    while (i < n) { // O(n)
        //Если текущий рейтинг равен предыдущему, просто переходим к следующему элементу
        if (ratings[i] == ratings[i - 1]) { // O(1) 
            i++; // O(1)
            continue; // O(1)
        }

        //Если текущий рейтинг больше предыдущего, начинаем обработку "пика". Увеличиваем currentPeak и добавляем его к totalCandies
        int currentPeak = 0; // O(1)
        while (i < n && ratings[i] > ratings[i - 1]) { // O(n)
            currentPeak++; // O(1)
            totalCandies += currentPeak; // O(1)
            i++; // O(1)
        }

        //Если достигли конца вектора, возвращаем totalCandies
        if (i == n) { // O(1)
            return totalCandies; // O(1)
        }

        //Если текущий рейтинг меньше предыдущего, начинаем обработку "долины". Увеличиваем currentValley и добавляем его к totalCandies
        int currentValley = 0; // O(1)
        while (i < n && ratings[i] < ratings[i - 1]) { // O(n)
            currentValley++; // O(1)
            totalCandies += currentValley; // O(1)
            i++; // O(1)
        }
        
        //Вычитаем минимальное значение между currentPeak и currentValley из totalCandies, чтобы минимизировать общее количество конфет
        totalCandies -= min(currentPeak, currentValley); // O(1)
    }

    return totalCandies; // O(1)
}

int main() {
    vector<int> test1 = { 1,2,2 };
    cout << candy(test1);
}

