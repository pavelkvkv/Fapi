#ifndef FAPI_HPP
#define FAPI_HPP

#include "ff.h"
#include "sem_mutex.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

/*Структура WAV заголовка (44 байта) – упаковка без выравнивания.
#pragma pack(push, 1)
struct wav_header {
    char     chunkID[4];       // "RIFF"
    uint32_t chunkSize;        // размер файла - 8 (будет обновляться)
    char     format[4];        // "WAVE"
    
    char     subchunk1ID[4];   // "fmt "
    uint32_t subchunk1Size;    // 16
    uint16_t audioFormat;      // 1 (PCM)
    uint16_t numChannels;      // 1
    uint32_t sampleRate;       // 16000
    uint32_t byteRate;         // 2 * 1 * 16000
    uint16_t blockAlign;       // 2
    uint16_t bitsPerSample;    // 16
    
    char     subchunk2ID[4];   // "data"
    uint32_t subchunk2Size;    // размер данных (будет обновляться)
};
#pragma pack(pop)*/

/*
struct index_entry_t {
    uint16_t index;    // порядковый номер в индексном файле
    uint32_t size;
    uint16_t type;     // тип файла (битовая маска из filetype_e)
    bool archived;     // файл из каталога rec скопирован в каталог arc
    bool deleted;      // запись не связана с файлом
    char filename[128]; // только имя файла, без пути
    struct AudioInfo {
        uint8_t channel;  // номер канала (если удалось распарсить)
        uint8_t year;     // год (относительно 2000)
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
        uint8_t codec;    // оставляем для совместимости
        uint16_t duration;
    } audio;
};*/

// структура index_entry_t определена в другом месте проекта.
struct index_entry_t;

class Fapi {
public:
    using AFILE = int; // кастомный дескриптор

    // Получение экземпляра (синглтон)
    static Fapi& instance();


    // Открытие файла.
    // Если файл уже открыт, возвращается существующий дескриптор.
    // path – путь к файлу, mode – режим открытия (FA_READ, FA_WRITE, FA_OPEN_EXISTING и т.п.).
    // timeout_ms – время автозакрытия (0 – не закрывать автоматически).
    // updateWavHeader – флаг обновления заголовка wav при закрытии.
    // Возвращает дескриптор (> 0) или отрицательное значение в случае ошибки.
    AFILE openFile(const char* path, BYTE mode, uint32_t timeout_ms = 0, bool updateWavHeader = false);

    // Открытие каталога.
    // Если каталог уже открыт, возвращается существующий дескриптор.
    // Возвращается универсальный дескриптор.
    AFILE openDirectory(const char* path);

    // Файловые операции – при каждом обращении к файлу сбрасывается таймер автозакрытия.
    void    close(AFILE handle);
    int     read(AFILE handle, void* buffer, UINT bytesToRead, UINT* bytesRead);
    int     write(AFILE handle, const void* buffer, UINT bytesToWrite, UINT* bytesWritten);
    uint32_t fileSize(AFILE handle);
    int     seek(AFILE handle, FSIZE_t offset);
    int     deleteFile(const char* path);
    int     renameFile(const char* oldName, const char* newName);

    // Быстрые файловые операции
    int readFile(const char *path, std::vector<uint8_t> &buffer);
    int readFile(const char *path, std::string &buffer);
    int writeFile(const char *path, const std::vector<uint8_t> &buffer);
    int writeFile(const char *path, const std::string &buffer);

    // Операции с каталогами
    int createPath(const char* path);       // создание каталога (рекурсивно)
    int deleteDirectory(const char* path);    // удаление каталога (рекурсивно, включая непустой)

   

    // Обёртки для функций FatFS: копирование файла и каталога.
    int copyFile(const char* from, const char* to);
    int copyDirectory(const char* from, const char* to, bool enc_conv);

    // Методы проверки и изменения состояния дескриптора файла.
    // При обращении к открытому файлу сбрасывается таймер автозакрытия.
    bool    isOpen(AFILE handle);
    bool    hasAutoCloseTimeout(AFILE handle);
    void    setAutoCloseTimeout(AFILE handle, uint32_t timeout_ms);
    uint32_t getAutoCloseTimeout(AFILE handle);
    void    setUpdateWavHeader(AFILE handle, bool flag);
    bool    getUpdateWavHeader(AFILE handle);

    // Мгновенное закрытие всех открытых файлов с режимом записи.
    void closeAllWriteFiles();

private:
    Fapi();
    ~Fapi();
    Fapi(const Fapi&) = delete;
    Fapi& operator=(const Fapi&) = delete;

    enum class DescriptorType { File, Directory };

    // Универсальный дескриптор для файла/каталога
    struct Descriptor {
        DescriptorType type;
        // Объединение для FatFS структур
        union {
            FIL file;
            DIR dir;
        };
        uint32_t    timeout_ms;         // 0 – автозакрытие не активно
        TickType_t  lastActivityTick;   // метка времени последней активности
        bool        updateWavHeader;    // нужно ли обновлять заголовок wav при закрытии
        bool        writeMode;          // открыт для записи
        int         mode_fatfs;
        std::string path;               // путь (для отладки или повторного открытия)

        Descriptor() : timeout_ms(0), lastActivityTick(0), updateWavHeader(false), writeMode(false) {}
    };

    // Хранилище дескрипторов
    std::unordered_map<AFILE, Descriptor> descriptors_;
    AFILE nextHandle_;

    // Приватный метод обновления заголовка wav (при закрытии файла)
    int updateWavHeader(Descriptor& desc);

    // FreeRTOS таск, который проверяет таймауты автозакрытия и закрывает файлы
    static void autoCloseTask(void* pvParameters);

    // Получение текущего тика (FreeRTOS)
    TickType_t getTickCount();

    // Удаление дескриптора из хранилища
    void removeDescriptor(AFILE handle);

	

    // Для перебора индексных записей
    std::string currentIndexDir_;
    AFILE indexFileHandle_ = -1;
};

extern "C" {
    // Внешняя функция генерации файла index.
    FRESULT generateIndex(const char* path);
}

#endif // FAPI_HPP
