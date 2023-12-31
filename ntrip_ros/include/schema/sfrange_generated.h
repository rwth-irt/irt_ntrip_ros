// automatically generated by the FlatBuffers compiler, do not modify


#ifndef FLATBUFFERS_GENERATED_SFRANGE_IRT_SFUSION_H_
#define FLATBUFFERS_GENERATED_SFRANGE_IRT_SFUSION_H_

#include "flatbuffers/flatbuffers.h"

// Ensure the included flatbuffers.h is the same version as when this file was
// generated, otherwise it may not be compatible.
static_assert(FLATBUFFERS_VERSION_MAJOR == 23 &&
              FLATBUFFERS_VERSION_MINOR == 1 &&
              FLATBUFFERS_VERSION_REVISION == 4,
             "Non-compatible flatbuffers version included");

#include "sfusion_generated.h"

namespace IRT {
namespace SFusion {

struct NovAtelRange;
struct NovAtelRangeBuilder;

struct NovAtelRange FLATBUFFERS_FINAL_CLASS : private ::flatbuffers::Table {
  typedef NovAtelRangeBuilder Builder;
  enum FlatBuffersVTableOffset FLATBUFFERS_VTABLE_UNDERLYING_TYPE {
    VT_SF_TIME = 4,
    VT_ANT_ID = 6,
    VT_HEADER_WEEK = 8,
    VT_HEADER_MS = 10,
    VT_NUM_OBS = 12,
    VT_PRN = 14,
    VT_GLOFREQ = 16,
    VT_PSR = 18,
    VT_PSR_STD = 20,
    VT_ADR = 22,
    VT_ADR_STD = 24,
    VT_DOPP = 26,
    VT_CN0 = 28,
    VT_LOCKTIME = 30,
    VT_CH_TR_STATUS = 32
  };
  const IRT::SFusion::SFusionTime *sf_time() const {
    return GetStruct<const IRT::SFusion::SFusionTime *>(VT_SF_TIME);
  }
  uint8_t ant_id() const {
    return GetField<uint8_t>(VT_ANT_ID, 0);
  }
  uint16_t header_week() const {
    return GetField<uint16_t>(VT_HEADER_WEEK, 0);
  }
  int32_t header_ms() const {
    return GetField<int32_t>(VT_HEADER_MS, 0);
  }
  uint32_t num_obs() const {
    return GetField<uint32_t>(VT_NUM_OBS, 0);
  }
  const ::flatbuffers::Vector<uint16_t> *prn() const {
    return GetPointer<const ::flatbuffers::Vector<uint16_t> *>(VT_PRN);
  }
  const ::flatbuffers::Vector<uint16_t> *glofreq() const {
    return GetPointer<const ::flatbuffers::Vector<uint16_t> *>(VT_GLOFREQ);
  }
  const ::flatbuffers::Vector<double> *psr() const {
    return GetPointer<const ::flatbuffers::Vector<double> *>(VT_PSR);
  }
  const ::flatbuffers::Vector<float> *psr_std() const {
    return GetPointer<const ::flatbuffers::Vector<float> *>(VT_PSR_STD);
  }
  const ::flatbuffers::Vector<double> *adr() const {
    return GetPointer<const ::flatbuffers::Vector<double> *>(VT_ADR);
  }
  const ::flatbuffers::Vector<float> *adr_std() const {
    return GetPointer<const ::flatbuffers::Vector<float> *>(VT_ADR_STD);
  }
  const ::flatbuffers::Vector<float> *dopp() const {
    return GetPointer<const ::flatbuffers::Vector<float> *>(VT_DOPP);
  }
  const ::flatbuffers::Vector<float> *cn0() const {
    return GetPointer<const ::flatbuffers::Vector<float> *>(VT_CN0);
  }
  const ::flatbuffers::Vector<float> *locktime() const {
    return GetPointer<const ::flatbuffers::Vector<float> *>(VT_LOCKTIME);
  }
  const ::flatbuffers::Vector<uint32_t> *ch_tr_status() const {
    return GetPointer<const ::flatbuffers::Vector<uint32_t> *>(VT_CH_TR_STATUS);
  }
  bool Verify(::flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyField<IRT::SFusion::SFusionTime>(verifier, VT_SF_TIME, 8) &&
           VerifyField<uint8_t>(verifier, VT_ANT_ID, 1) &&
           VerifyField<uint16_t>(verifier, VT_HEADER_WEEK, 2) &&
           VerifyField<int32_t>(verifier, VT_HEADER_MS, 4) &&
           VerifyField<uint32_t>(verifier, VT_NUM_OBS, 4) &&
           VerifyOffset(verifier, VT_PRN) &&
           verifier.VerifyVector(prn()) &&
           VerifyOffset(verifier, VT_GLOFREQ) &&
           verifier.VerifyVector(glofreq()) &&
           VerifyOffset(verifier, VT_PSR) &&
           verifier.VerifyVector(psr()) &&
           VerifyOffset(verifier, VT_PSR_STD) &&
           verifier.VerifyVector(psr_std()) &&
           VerifyOffset(verifier, VT_ADR) &&
           verifier.VerifyVector(adr()) &&
           VerifyOffset(verifier, VT_ADR_STD) &&
           verifier.VerifyVector(adr_std()) &&
           VerifyOffset(verifier, VT_DOPP) &&
           verifier.VerifyVector(dopp()) &&
           VerifyOffset(verifier, VT_CN0) &&
           verifier.VerifyVector(cn0()) &&
           VerifyOffset(verifier, VT_LOCKTIME) &&
           verifier.VerifyVector(locktime()) &&
           VerifyOffset(verifier, VT_CH_TR_STATUS) &&
           verifier.VerifyVector(ch_tr_status()) &&
           verifier.EndTable();
  }
};

struct NovAtelRangeBuilder {
  typedef NovAtelRange Table;
  ::flatbuffers::FlatBufferBuilder &fbb_;
  ::flatbuffers::uoffset_t start_;
  void add_sf_time(const IRT::SFusion::SFusionTime *sf_time) {
    fbb_.AddStruct(NovAtelRange::VT_SF_TIME, sf_time);
  }
  void add_ant_id(uint8_t ant_id) {
    fbb_.AddElement<uint8_t>(NovAtelRange::VT_ANT_ID, ant_id, 0);
  }
  void add_header_week(uint16_t header_week) {
    fbb_.AddElement<uint16_t>(NovAtelRange::VT_HEADER_WEEK, header_week, 0);
  }
  void add_header_ms(int32_t header_ms) {
    fbb_.AddElement<int32_t>(NovAtelRange::VT_HEADER_MS, header_ms, 0);
  }
  void add_num_obs(uint32_t num_obs) {
    fbb_.AddElement<uint32_t>(NovAtelRange::VT_NUM_OBS, num_obs, 0);
  }
  void add_prn(::flatbuffers::Offset<::flatbuffers::Vector<uint16_t>> prn) {
    fbb_.AddOffset(NovAtelRange::VT_PRN, prn);
  }
  void add_glofreq(::flatbuffers::Offset<::flatbuffers::Vector<uint16_t>> glofreq) {
    fbb_.AddOffset(NovAtelRange::VT_GLOFREQ, glofreq);
  }
  void add_psr(::flatbuffers::Offset<::flatbuffers::Vector<double>> psr) {
    fbb_.AddOffset(NovAtelRange::VT_PSR, psr);
  }
  void add_psr_std(::flatbuffers::Offset<::flatbuffers::Vector<float>> psr_std) {
    fbb_.AddOffset(NovAtelRange::VT_PSR_STD, psr_std);
  }
  void add_adr(::flatbuffers::Offset<::flatbuffers::Vector<double>> adr) {
    fbb_.AddOffset(NovAtelRange::VT_ADR, adr);
  }
  void add_adr_std(::flatbuffers::Offset<::flatbuffers::Vector<float>> adr_std) {
    fbb_.AddOffset(NovAtelRange::VT_ADR_STD, adr_std);
  }
  void add_dopp(::flatbuffers::Offset<::flatbuffers::Vector<float>> dopp) {
    fbb_.AddOffset(NovAtelRange::VT_DOPP, dopp);
  }
  void add_cn0(::flatbuffers::Offset<::flatbuffers::Vector<float>> cn0) {
    fbb_.AddOffset(NovAtelRange::VT_CN0, cn0);
  }
  void add_locktime(::flatbuffers::Offset<::flatbuffers::Vector<float>> locktime) {
    fbb_.AddOffset(NovAtelRange::VT_LOCKTIME, locktime);
  }
  void add_ch_tr_status(::flatbuffers::Offset<::flatbuffers::Vector<uint32_t>> ch_tr_status) {
    fbb_.AddOffset(NovAtelRange::VT_CH_TR_STATUS, ch_tr_status);
  }
  explicit NovAtelRangeBuilder(::flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  ::flatbuffers::Offset<NovAtelRange> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = ::flatbuffers::Offset<NovAtelRange>(end);
    return o;
  }
};

inline ::flatbuffers::Offset<NovAtelRange> CreateNovAtelRange(
    ::flatbuffers::FlatBufferBuilder &_fbb,
    const IRT::SFusion::SFusionTime *sf_time = nullptr,
    uint8_t ant_id = 0,
    uint16_t header_week = 0,
    int32_t header_ms = 0,
    uint32_t num_obs = 0,
    ::flatbuffers::Offset<::flatbuffers::Vector<uint16_t>> prn = 0,
    ::flatbuffers::Offset<::flatbuffers::Vector<uint16_t>> glofreq = 0,
    ::flatbuffers::Offset<::flatbuffers::Vector<double>> psr = 0,
    ::flatbuffers::Offset<::flatbuffers::Vector<float>> psr_std = 0,
    ::flatbuffers::Offset<::flatbuffers::Vector<double>> adr = 0,
    ::flatbuffers::Offset<::flatbuffers::Vector<float>> adr_std = 0,
    ::flatbuffers::Offset<::flatbuffers::Vector<float>> dopp = 0,
    ::flatbuffers::Offset<::flatbuffers::Vector<float>> cn0 = 0,
    ::flatbuffers::Offset<::flatbuffers::Vector<float>> locktime = 0,
    ::flatbuffers::Offset<::flatbuffers::Vector<uint32_t>> ch_tr_status = 0) {
  NovAtelRangeBuilder builder_(_fbb);
  builder_.add_ch_tr_status(ch_tr_status);
  builder_.add_locktime(locktime);
  builder_.add_cn0(cn0);
  builder_.add_dopp(dopp);
  builder_.add_adr_std(adr_std);
  builder_.add_adr(adr);
  builder_.add_psr_std(psr_std);
  builder_.add_psr(psr);
  builder_.add_glofreq(glofreq);
  builder_.add_prn(prn);
  builder_.add_num_obs(num_obs);
  builder_.add_header_ms(header_ms);
  builder_.add_sf_time(sf_time);
  builder_.add_header_week(header_week);
  builder_.add_ant_id(ant_id);
  return builder_.Finish();
}

inline ::flatbuffers::Offset<NovAtelRange> CreateNovAtelRangeDirect(
    ::flatbuffers::FlatBufferBuilder &_fbb,
    const IRT::SFusion::SFusionTime *sf_time = nullptr,
    uint8_t ant_id = 0,
    uint16_t header_week = 0,
    int32_t header_ms = 0,
    uint32_t num_obs = 0,
    const std::vector<uint16_t> *prn = nullptr,
    const std::vector<uint16_t> *glofreq = nullptr,
    const std::vector<double> *psr = nullptr,
    const std::vector<float> *psr_std = nullptr,
    const std::vector<double> *adr = nullptr,
    const std::vector<float> *adr_std = nullptr,
    const std::vector<float> *dopp = nullptr,
    const std::vector<float> *cn0 = nullptr,
    const std::vector<float> *locktime = nullptr,
    const std::vector<uint32_t> *ch_tr_status = nullptr) {
  auto prn__ = prn ? _fbb.CreateVector<uint16_t>(*prn) : 0;
  auto glofreq__ = glofreq ? _fbb.CreateVector<uint16_t>(*glofreq) : 0;
  auto psr__ = psr ? _fbb.CreateVector<double>(*psr) : 0;
  auto psr_std__ = psr_std ? _fbb.CreateVector<float>(*psr_std) : 0;
  auto adr__ = adr ? _fbb.CreateVector<double>(*adr) : 0;
  auto adr_std__ = adr_std ? _fbb.CreateVector<float>(*adr_std) : 0;
  auto dopp__ = dopp ? _fbb.CreateVector<float>(*dopp) : 0;
  auto cn0__ = cn0 ? _fbb.CreateVector<float>(*cn0) : 0;
  auto locktime__ = locktime ? _fbb.CreateVector<float>(*locktime) : 0;
  auto ch_tr_status__ = ch_tr_status ? _fbb.CreateVector<uint32_t>(*ch_tr_status) : 0;
  return IRT::SFusion::CreateNovAtelRange(
      _fbb,
      sf_time,
      ant_id,
      header_week,
      header_ms,
      num_obs,
      prn__,
      glofreq__,
      psr__,
      psr_std__,
      adr__,
      adr_std__,
      dopp__,
      cn0__,
      locktime__,
      ch_tr_status__);
}

inline const IRT::SFusion::NovAtelRange *GetNovAtelRange(const void *buf) {
  return ::flatbuffers::GetRoot<IRT::SFusion::NovAtelRange>(buf);
}

inline const IRT::SFusion::NovAtelRange *GetSizePrefixedNovAtelRange(const void *buf) {
  return ::flatbuffers::GetSizePrefixedRoot<IRT::SFusion::NovAtelRange>(buf);
}

inline bool VerifyNovAtelRangeBuffer(
    ::flatbuffers::Verifier &verifier) {
  return verifier.VerifyBuffer<IRT::SFusion::NovAtelRange>(nullptr);
}

inline bool VerifySizePrefixedNovAtelRangeBuffer(
    ::flatbuffers::Verifier &verifier) {
  return verifier.VerifySizePrefixedBuffer<IRT::SFusion::NovAtelRange>(nullptr);
}

inline void FinishNovAtelRangeBuffer(
    ::flatbuffers::FlatBufferBuilder &fbb,
    ::flatbuffers::Offset<IRT::SFusion::NovAtelRange> root) {
  fbb.Finish(root);
}

inline void FinishSizePrefixedNovAtelRangeBuffer(
    ::flatbuffers::FlatBufferBuilder &fbb,
    ::flatbuffers::Offset<IRT::SFusion::NovAtelRange> root) {
  fbb.FinishSizePrefixed(root);
}

}  // namespace SFusion
}  // namespace IRT

#endif  // FLATBUFFERS_GENERATED_SFRANGE_IRT_SFUSION_H_
