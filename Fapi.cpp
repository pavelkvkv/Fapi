#include "Fapi.hpp"

#include "sem_mutex.hpp"

#include <string>
#include <vector>

#include <tuple>
#include <algorithm>

//#include "IndexController.hpp"

extern "C"
{
#define TAG "Fapi"
#include "log.h"
#include "global.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ff.h"
#include "clock.h"
}

// Реализация синглтона
Fapi &Fapi::instance()
{
	Waitfor(SysState.SD_ok);
	static Fapi instance;
	return instance;
}

Fapi::Fapi() : nextHandle_(1)
{
	Sem::create_if_not_exist(Sem::ResourceType::MUTEX, Sem::ResourceKey::FATFS_MUT, 1000, Sem::TimeoutAction::LogErrorWaitAgainThenAbort);
	xTaskCreate(autoCloseTask, "FapiAutoClose", 1024, this, tskIDLE_PRIORITY + 1, nullptr);
	logI("Fapi created");
}

Fapi::~Fapi()
{
	logI("Fapi destroyed");
	for (auto &pair : descriptors_)
	{
		if (pair.second.type == DescriptorType::File)
		{
			f_close(&pair.second.file);
		}
		else
		{
			f_closedir(&pair.second.dir);
		}
	}
	descriptors_.clear();
}

Fapi::AFILE Fapi::openFile(const char *path, BYTE mode, uint32_t timeout_ms, bool updateWavHeaderFlag)
{
    logI("openFile path=%s mode=%u timeout=%u wav=%d", path, mode, timeout_ms, updateWavHeaderFlag);

    // Определяем, запрашивается ли запись.
    bool reqWrite = (mode & FA_WRITE) ? true : false;

    {
        SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, -1);

        // Если файл уже открыт, ищем по имени и режиму доступа
        for (auto &pair : descriptors_)
        {
            if (pair.second.type == DescriptorType::File && pair.second.path == path)
            {
                bool existingWrite = pair.second.writeMode;
                // Если режимы совпадают
                if (existingWrite == reqWrite)
                {
                    if (reqWrite)
                    {
                        // Двойное открытие на запись запрещено
                        logE("openFile error: file %s already open for writing, handle=%u", path, pair.first);
                        return -1;
                    }
                    else
                    {
                        // Файл уже открыт для чтения – возвращаем существующий дескриптор
                        if (pair.second.timeout_ms != 0)
                        {
                            pair.second.lastActivityTick = getTickCount();
                        }
                        logD("File already open for reading, handle=%u", pair.first);
                        return pair.first;
                    }
                }
            }
        }
    }

    // Создание нового дескриптора
    Descriptor desc;
    desc.type              = DescriptorType::File;
    desc.timeout_ms        = timeout_ms;
    desc.lastActivityTick  = getTickCount();
    desc.updateWavHeader   = updateWavHeaderFlag;
    desc.writeMode         = reqWrite;
    desc.path              = path;

    // Если файл открыт для записи, создаём необходимые каталоги
    if (desc.writeMode)
    {
        int ret = createPath(path);
        if (ret != 0)
        {
            logW("createPath failed for %s with code %d", path, ret);
        }
    }
    
    AFILE handle = 0;
    {
		FRESULT res = FR_OK;
		FIL file;
        {
			SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, -1);
        	res = f_open(&file, path, mode);
		}
        // Если получена ошибка FR_LOCKED, ждём 500 мс и пробуем ещё раз
        if (res == FR_LOCKED)
        {
            vTaskDelay(pdMS_TO_TICKS(500));
			{
			SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, -1);
            res = f_open(&file, path, mode);
			}
        }
        if (res != FR_OK)
        {
            logE("f_open failed path=%s res=%d", path, res);
            return -1;
        }
        desc.file = file;
        handle    = nextHandle_++;
        descriptors_[handle] = desc;
    }
    logD("File opened, handle=%u", handle);
    return handle;
}


Fapi::AFILE Fapi::openDirectory(const char *path)
{
	logI("openDirectory path=%s", path);
	SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, -1);

	// Если каталог уже открыт, возвращаем существующий дескриптор и сбрасываем таймер
	for (auto &pair : descriptors_)
	{
		if (pair.second.type == DescriptorType::Directory && pair.second.path == path)
		{
			if (pair.second.timeout_ms != 0)
			{
				pair.second.lastActivityTick = getTickCount();
			}
			logD("Directory already open, handle=%u", pair.first);
			return pair.first;
		}
	}

	Descriptor desc = {};
	desc.type			  = DescriptorType::Directory;
	desc.timeout_ms		  = 0;
	desc.lastActivityTick = getTickCount();
	desc.updateWavHeader  = false;
	desc.writeMode		  = false;
	desc.path			  = path;
	DIR dir;
	FRESULT res = f_opendir(&dir, path);
	if (res != FR_OK)
	{
		logW("f_opendir failed path=%s res=%d", path, res);
		return -1;
	}
	desc.dir			 = dir;
	AFILE handle		 = nextHandle_++;
	descriptors_[handle] = desc;
	logD("Directory opened, handle=%u", handle);
	return handle;
}

void Fapi::close(AFILE handle)
{
	logI("close handle=%u", handle);
	SEM_LOCK_OR_RETURN_VOID(Sem::ResourceKey::FATFS_MUT);
	auto it = descriptors_.find(handle);
	if (it == descriptors_.end())
	{
		logW("close failed: handle not found");
		return;
	}
	Descriptor &desc = it->second;
	if (desc.type == DescriptorType::File)
	{
		if (desc.updateWavHeader && desc.writeMode)
		{
			updateWavHeader(desc);
		}
		if (desc.writeMode)
		{
			// std::tuple<uint32_t, uint32_t, uint32_t> indexData = getIndexData(desc);
			// IndexTask task									   = {IndexTaskType::AddFile, desc.path, std::get<0>(indexData), std::get<1>(indexData), std::get<2>(indexData)};
			// IndexController::getInstance().enqueueTask(task);
		}
		f_close(&desc.file);
		logD("File closed, handle=%u", handle);
	}
	else if (desc.type == DescriptorType::Directory)
	{
		f_closedir(&desc.dir);
		logD("Directory closed, handle=%u", handle);
	}
	descriptors_.erase(it);
}

int Fapi::read(AFILE handle, void *buffer, UINT bytesToRead, UINT *bytesRead)
{
	logI("read handle=%u bytesToRead=%u", handle, bytesToRead);
	SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, -1);
	auto it = descriptors_.find(handle);
	if (it == descriptors_.end() || it->second.type != DescriptorType::File)
	{
		logW("read failed: invalid handle or not a file");
		return -1;
	}
	Descriptor &desc	  = it->second;
	desc.lastActivityTick = getTickCount();
	FRESULT res			  = f_read(&desc.file, buffer, bytesToRead, bytesRead);
	if (res != FR_OK)
		logW("f_read failed res=%d", res);
	return (res == FR_OK) ? 0 : res;
}

int Fapi::write(AFILE handle, const void *buffer, UINT bytesToWrite, UINT *bytesWritten)
{
	logI("write handle=%u bytesToWrite=%u", handle, bytesToWrite);
	SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, -1);
	auto it = descriptors_.find(handle);
	if (it == descriptors_.end() || it->second.type != DescriptorType::File)
	{
		logW("write failed: invalid handle or not a file");
		return -1;
	}
	Descriptor &desc	  = it->second;
	desc.lastActivityTick = getTickCount();
	FRESULT res			  = f_write(&desc.file, buffer, bytesToWrite, bytesWritten);
	if (res != FR_OK)
		logE("f_write failed res=%d", res);
	return (res == FR_OK) ? 0 : res;
}

uint32_t Fapi::fileSize(AFILE handle)
{
	logI("fileSize: handle=%d", handle);
	SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, 0);
	auto it = descriptors_.find(handle);
	if (it == descriptors_.end() || it->second.type != DescriptorType::File)
	{
		logW("fileSize failed: invalid handle or not a file: %d", handle);
		return 0;
	}
	// Сброс таймера автозакрытия при обращении
	it->second.lastActivityTick = getTickCount();
	uint32_t size				= f_size(&it->second.file);
	logD("fileSize=%u", size);
	return size;
}

int Fapi::seek(AFILE handle, FSIZE_t offset)
{
	logI("seek: handle=%d, offset=%u", handle, (uint32_t)offset);
	SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, -1);
	auto it = descriptors_.find(handle);
	if (it == descriptors_.end() || it->second.type != DescriptorType::File)
	{
		logW("seek failed: invalid handle or not a file: %d", handle);
		return -1;
	}
	Descriptor &desc	  = it->second;
	desc.lastActivityTick = getTickCount();
	FRESULT res			  = f_lseek(&desc.file, offset);
	if (res != FR_OK)
	{
		logE("f_lseek failed with code %d", res);
	}
	return (res == FR_OK) ? 0 : res;
}

int Fapi::deleteFile(const char *path)
{
	logI("deleteFile: path=%s", path);
	SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, -1);
	FRESULT res = f_unlink(path);
	if (res != FR_OK)
	{
		logW("f_unlink failed for %s with code %d", path, res);
	}
	return (res == FR_OK) ? 0 : res;
}

int Fapi::renameFile(const char *oldName, const char *newName)
{
	logI("renameFile: %s -> %s", oldName, newName);
	SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, -1);
	FRESULT res = f_rename(oldName, newName);
	if (res != FR_OK)
	{
		logW("f_rename failed with code %d", res);
	}
	return (res == FR_OK) ? 0 : res;
}

int Fapi::createPath(const char *path)
{
    if (path == nullptr || strlen(path) == 0)
    {
        logE("createPath: некорректный путь");
        return -1;
    }

    // Приводим входной путь к std::string для удобства обработки
    std::string p(path);

    // Если путь не заканчивается разделителем, отсекаем часть после последнего разделителя.
    if (!p.empty() && p.back() != '/' && p.back() != '\\')
    {
        size_t pos = p.find_last_of("/\\");
        if (pos != std::string::npos)
        {
            p = p.substr(0, pos + 1); // оставляем разделитель в конце
        }
        else
        {
            logE("createPath: разделитель в пути не найден: %s", path);
            return -1;
        }
    }

    logI("createPath: путь=%s", p.c_str());
    SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, -1);

    // Создание промежуточных директорий
    size_t pos = 0;
    while ((pos = p.find_first_of("/\\", pos)) != std::string::npos)
    {
        std::string sub = p.substr(0, pos);
        if (!sub.empty())
        {
            logD("Создание промежуточной директории: %s", sub.c_str());
            f_mkdir(sub.c_str());
        }
        pos++;
    }

    // Создание конечной директории
    FRESULT res = f_mkdir(p.c_str());
    if (res != FR_OK && res != FR_EXIST)
    {
        logW("f_mkdir не удалось создать %s, код ошибки %d", p.c_str(), res);
    }
    return (res == FR_OK || res == FR_EXIST) ? 0 : res;
}


int Fapi::deleteDirectory(const char *path)
{
	logI("deleteDirectory: path=%s", path);
	DIR dir;
	FRESULT res;

	{
		SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, -1);
		res = f_opendir(&dir, path);
		if (res != FR_OK)
		{
			logW("f_opendir failed for %s with code %d", path, res);
			return res;
		}
	}

	std::vector<std::string> subdirs;
	std::vector<std::string> files;
	FILINFO fno;
	while (true)
	{
		{
			SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, -1);
			res = f_readdir(&dir, &fno);
		}
		if (res != FR_OK)
		{
			logW("f_readdir failed with code %d", res);
			break;
		}
		if (fno.fname[0] == 0)
			break;
		std::string fullPath = std::string(path) + "/" + fno.fname;
		if (fno.fattrib & AM_DIR)
		{
			logD("Subdir found: %s", fullPath.c_str());
			subdirs.push_back(fullPath);
		}
		else
		{
			logD("File found: %s", fullPath.c_str());
			files.push_back(fullPath);
		}
	}

	{
		SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, -1);
		f_closedir(&dir);
	}

	for (auto &file : files)
	{
		logD("Deleting file: %s", file.c_str());
		SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, -1);
		f_unlink(file.c_str());
	}

	for (auto &subdir : subdirs)
	{
		deleteDirectory(subdir.c_str());
	}

	{
		SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, -1);
		res = f_unlink(path);
	}

	if (res != FR_OK)
	{
		logW("f_unlink (directory) failed for %s with code %d", path, res);
	}
	return (res == FR_OK) ? 0 : res;
}

int Fapi::copyFile(const char *from, const char *to)
{
	logI("copyFile: from=%s, to=%s", from, to);
	SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, -1);
	FRESULT res = f_copy(from, to);
	if (res != FR_OK)
	{
		logE("f_copy failed from %s to %s with code %d", from, to, res);
	}
	return (res == FR_OK) ? 0 : res;
}

int Fapi::copyDirectory(const char *from, const char *to, bool enc_conv)
{
	logI("copyDirectory: from=%s, to=%s, enc_conv=%d", from, to, enc_conv);
	SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, -1);
	FRESULT res = f_dircopy(from, to, enc_conv);
	if (res != FR_OK)
	{
		logE("f_dircopy failed with code %d", res);
	}
	return (res == FR_OK) ? 0 : res;
}

bool Fapi::isOpen(AFILE handle)
{
	logI("isOpen: handle=%d", handle);
	SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, false);
	auto it = descriptors_.find(handle);
	if (it != descriptors_.end())
	{
		if (it->second.timeout_ms != 0)
			it->second.lastActivityTick = getTickCount();
		logD("isOpen result: true");
		return true;
	}
	logD("isOpen result: false");
	return false;
}

bool Fapi::hasAutoCloseTimeout(AFILE handle)
{
	logI("hasAutoCloseTimeout: handle=%d", handle);
	SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, false);
	auto it = descriptors_.find(handle);
	if (it == descriptors_.end() || it->second.type != DescriptorType::File)
	{
		logW("hasAutoCloseTimeout failed: invalid handle or not a file: %d", handle);
		return false;
	}
	it->second.lastActivityTick = getTickCount();
	bool result					= it->second.timeout_ms > 0;
	logD("timeout_ms=%u", it->second.timeout_ms);
	return result;
}

void Fapi::setAutoCloseTimeout(AFILE handle, uint32_t timeout_ms)
{
	logI("setAutoCloseTimeout: handle=%d, timeout_ms=%u", handle, timeout_ms);
	SEM_LOCK_OR_RETURN_VOID(Sem::ResourceKey::FATFS_MUT);
	auto it = descriptors_.find(handle);
	if (it != descriptors_.end() && it->second.type == DescriptorType::File)
	{
		it->second.timeout_ms		= timeout_ms;
		it->second.lastActivityTick = getTickCount();
		logD("Timeout updated");
	}
	else
	{
		logW("setAutoCloseTimeout failed: invalid handle or not a file: %d", handle);
	}
}

uint32_t Fapi::getAutoCloseTimeout(AFILE handle)
{
	logI("getAutoCloseTimeout: handle=%d", handle);
	SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, 0);
	auto it = descriptors_.find(handle);
	if (it == descriptors_.end() || it->second.type != DescriptorType::File)
	{
		logW("getAutoCloseTimeout failed: invalid handle or not a file: %d", handle);
		return 0;
	}
	it->second.lastActivityTick = getTickCount();
	logD("timeout_ms=%u", it->second.timeout_ms);
	return it->second.timeout_ms;
}

void Fapi::setUpdateWavHeader(AFILE handle, bool flag)
{
	logI("setUpdateWavHeader: handle=%d, flag=%d", handle, flag);
	SEM_LOCK_OR_RETURN_VOID(Sem::ResourceKey::FATFS_MUT);
	auto it = descriptors_.find(handle);
	if (it != descriptors_.end() && it->second.type == DescriptorType::File)
	{
		it->second.updateWavHeader	= flag;
		it->second.lastActivityTick = getTickCount();
		logD("Update flag set");
	}
	else
	{
		logW("setUpdateWavHeader failed: invalid handle or not a file: %d", handle);
	}
}

bool Fapi::getUpdateWavHeader(AFILE handle)
{
	logI("getUpdateWavHeader: handle=%d", handle);
	SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, false);
	auto it = descriptors_.find(handle);
	if (it == descriptors_.end() || it->second.type != DescriptorType::File)
	{
		logW("getUpdateWavHeader failed: invalid handle or not a file: %d", handle);
		return false;
	}
	it->second.lastActivityTick = getTickCount();
	logD("updateWavHeader=%d", it->second.updateWavHeader);
	return it->second.updateWavHeader;
}

void Fapi::closeAllWriteFiles()
{
	logI("closeAllWriteFiles");
	std::vector<AFILE> handlesToClose;
	{
		SEM_LOCK_OR_RETURN_VOID(Sem::ResourceKey::FATFS_MUT);
		
		for (auto &pair : descriptors_)
		{
			if (pair.second.type == DescriptorType::File && pair.second.writeMode)
			{
				logD("Marked for close: handle=%d", pair.first);
				handlesToClose.push_back(pair.first);
			}
		}
	}
	for (auto handle : handlesToClose)
	{
		close(handle);
	}
	logI("closeAllWriteFiles done");
}

int Fapi::updateWavHeader(Descriptor &desc)
{
	// Предполагается, что мьютекс уже захвачен.
	DWORD fileSize = f_size(&desc.file);
	if (fileSize < sizeof(wav_header))
		return -1;
	uint32_t chunkSize	   = fileSize - 8;
	uint32_t subchunk2Size = fileSize - sizeof(wav_header);

	// Инициализация стандартного заголовка
	static wav_header wav_header_common = {
		{'R', 'I', 'F', 'F'},
		0, // будет обновлено
		{'W', 'A', 'V', 'E'},
		{'f', 'm', 't', ' '},
		16,
		1,
		1,
		16000,
		2 * 1 * 16000,
		2,
		16,
		{'d', 'a', 't', 'a'},
		0 // будет обновлено
	};
	wav_header header = wav_header_common;
	header.wav_size	  = chunkSize;
	header.data_size  = subchunk2Size;

	UINT bw;
	FRESULT res;
	res = f_lseek(&desc.file, 0);
	if (res != FR_OK)
		return res;
	res = f_write(&desc.file, &header, sizeof(header), &bw);
	return (res == FR_OK && bw == sizeof(header)) ? 0 : res;
}

void Fapi::autoCloseTask(void *pvParameters)
{
	logI("autoCloseTask started");

	Fapi *self = static_cast<Fapi *>(pvParameters);
	while (true)
	{
		vTaskDelay(pdMS_TO_TICKS(1000)); // проверяем раз в 1 секунду
        std::vector<AFILE> toClose;
        {
		SEM_LOCK_OR_RETURN_VOID(Sem::ResourceKey::FATFS_MUT);
		TickType_t currentTick = self->getTickCount();
		
		for (auto &pair : self->descriptors_)
		{
			if (pair.second.type == DescriptorType::File && pair.second.timeout_ms > 0)
			{
				if (((currentTick - pair.second.lastActivityTick) * portTICK_PERIOD_MS) >= pair.second.timeout_ms)
				{
					toClose.push_back(pair.first);
				}
			}
		}
        }
		// Выходим из критической секции и закрываем просроченные файлы
		for (auto handle : toClose)
		{
			self->close(handle);
		}
	}
}

TickType_t Fapi::getTickCount()
{
	return xTaskGetTickCount();
}

int Fapi::readFile(const char *path, std::vector<uint8_t> &buffer)
{
	logI("Reading full file %s", path);
	FIL file;
	FRESULT res;
	{
		SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, -1);
		res = f_open(&file, path, FA_READ);
		if (res != FR_OK)
			return res;
	}
	DWORD size;
	{
		SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, -1);
		size = f_size(&file);
	}
	buffer.resize(size);
	UINT br;
	{
		SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, -1);
		res = f_read(&file, buffer.data(), size, &br);
		f_close(&file);
	}
	return (res == FR_OK && br == size) ? 0 : res;
}

// Тоже самое, но читаем в строку
int Fapi::readFile(const char *path, std::string &buffer)
{
	std::vector<uint8_t> buf;
	int res = readFile(path, buf);
	buffer.assign(buf.begin(), buf.end());
	return res;
}

int Fapi::writeFile(const char *path, const std::vector<uint8_t> &buffer)
{
	logI("Writing full file %s", path);
	FIL file;
	FRESULT res;
	{
		SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, -1);
		res = f_open(&file, path, FA_WRITE | FA_CREATE_ALWAYS);
		if (res != FR_OK)
			return res;
	}
	UINT bw;
	{
		SEM_LOCK_OR_RETURN(Sem::ResourceKey::FATFS_MUT, -1);
		res = f_write(&file, buffer.data(), buffer.size(), &bw);
		f_close(&file);
	}
	return (res == FR_OK && bw == buffer.size()) ? 0 : res;
}

int Fapi::writeFile(const char *path, const std::string &buffer)
{
	std::vector<uint8_t> buf(buffer.begin(), buffer.end());
	return writeFile(path, buf);
}

